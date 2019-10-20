#Use Python 2.X

#Use the following settings.json file (C:\Users\MyUser\Documents\AirSim)
#{
#  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
#  "SettingsVersion": 1.2,
#  "CameraDefaults": {
#      "CaptureSettings": [
#        {
#          "ImageType": 0,
#          "Width": 640,
#          "Height": 480,
#          "FOV_Degrees": 90,
#          "AutoExposureSpeed": 100,
#          "MotionBlurAmount": 0
#        }
#    ]
#  },
#  "SimMode": "ComputerVision",
#  "LocalHostIp": "192.168.10.21"
#}

import airsim

import time
from pymavlink import mavutil
import numpy
import cv2
from threading import Thread
from queue import Queue
import pygeodesy
import openCVUtils as util
import pdb  #for debugging - use pdb.set_trace() to set breakpoints inside threads

UseSimulator = True     #set to false for real flight   #TODO: make command line argument

#Settings:
FMV_DESTINATION = '192.168.10.21'   #AKA: Downlink video destination
FMV_PORT = '50006'
MAVLINK_C2_GCS = 'udpout:192.168.10.21:50005'
CAMERA_FOV = 500             #(unitless) determined experimentally

#Settings - Simulation only:
AIRSIM_SERVER = '192.168.10.21'
MAVLINK_HIGHSPEED_AUTOPILOT = 'tcp:192.168.10.10:5762'
MAVLINK_C2_AUTOPILOT = 'tcp:192.168.10.10:5763'
cameraType = airsim.ImageType.Scene         #EO color camera
#cameraType = airsim.ImageType.Infrared     #Thermal Camera TODO: fix frame size in AirSim settings.json file

#Mavlink C2 data
hud_airspeed = 0.0    #pitot static airspeed (m/s)
hud_altitude = 0.0    #absolute baro altitude (m MSL)
yaw = 0.0         #IMU heading (magnetometer) (deg true)
pitch = 0.0       #(deg)
roll = 0.0        #(deg)
global_lat = 0.0
global_lon = 0.0
global_alt = 0.0
global_relative_alt = 0.0
home_lat = 0.0      #Note: This mesage not typically sent wihout a prior request
home_lon = 0.0
home_alt = 0.0      #m MSL
mode = 0
waypoints = []        #list of tuples containing known waypoints
waypoint_quantity = 0 #the number of known waypoints as reported by the autopilot, if this is != len(waypoints) that means some waypoints are missing

shutdownMavlinkUDP = False
shutdownMavlinkTCP = False

#Simulation specific pose
pitchSim = 0.0
rollSim = 0.0
yawSim = 0.0
xSim = 0.0
ySim = 0.0
zSim = 0.0

frameCount = 0
startTime=time.clock()
fps = 0

def flightMode(i):
    switch={
        mavutil.ardupilotmega.PLANE_MODE_MANUAL:'MANUAL',
        mavutil.ardupilotmega.PLANE_MODE_FLY_BY_WIRE_A:'FBWA',
        mavutil.ardupilotmega.PLANE_MODE_FLY_BY_WIRE_B:'FBWB',
        mavutil.ardupilotmega.PLANE_MODE_CRUISE:'CRUISE',
        mavutil.ardupilotmega.PLANE_MODE_AUTO:'AUTO',
        mavutil.ardupilotmega.PLANE_MODE_RTL:'RTL',
        mavutil.ardupilotmega.PLANE_MODE_LOITER:'LOITER',
        mavutil.ardupilotmega.PLANE_MODE_GUIDED:'GUIDED'}
    return switch.get(i,"INVALID")

#Warning: Errors in Gstreamer pipes may silentley fail (escpically on Linux).
#Check pipes using the command line first

## Set Video In / Out ##
if UseSimulator:    #for Linux PC
    #Video Input - from Airsim Server 
    client = airsim.VehicleClient(AIRSIM_SERVER)
    print("Waiting for AirSim Server") 
    client.confirmConnection()
    print("Connected to Airsim Server")

    client.simEnableWeather(True)   #Enable Dynamic Weather - optional

    #Set output encoder to use software encoder    
    H264_ENCODER = 'x264enc bitrate=300 tune=zerolatency'   #NOTE BITRATE (kbps)   

else:   #for Nvidia Nano

    #Video Input - USB camera
    #Odroid Camera:
    #cap = cv2.VideoCapture('v4l2src device=/dev/video0 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=(fraction)30/1 ! jpegdec ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    #Lepoard Camera:
    cap = cv2.VideoCapture('v4l2src device=/dev/video0 ! video/x-raw, format=(string)YUY2, width=(int)1920, height=(int)1080, framerate=(fraction)30/1 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

    #Set output encoder to use hardware (Nvidia Nano)
    H264_ENCODER = 'omxh264enc control-rate=2 bitrate=400000' #NOTE BITRATE (bps)

   
#Create Video Output Pipe
out = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw, format=(string)I420  ! videorate ! video/x-raw, framerate=(fraction)5/1 ! '+H264_ENCODER+' ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay mtu=1400 ! udpsink host='+FMV_DESTINATION+' port='+FMV_PORT+' sync=false async=false', cv2.CAP_GSTREAMER,0,5,(640,480), True)

#Error checking for output pipe only
#TODO: Add error checking for input pipe
if (out.isOpened()):
    print("Output pipe Created Sucessfully")
else:
    print("Output pipe Creation Failed")

#Mavlink high speed link for driving Unreal sim pose
#Not used if UseSimulator=False
#Ardupilot setup:
#Add SR1 rate setting to settings
#Note: TCP 5762 = SR1 in settings
# Use Mission planner to set in the meantime
# Set SR1 rates to be SR1_EXTRA1 = SR1_POSITION = 20
# SR1_PARAMS = 10 (not entirely necessary)
# Set other SR1 rates to zero
class MavlinkHighSpeedTCP(Thread):

    def __init__(self, queue):
        Thread.__init__(self)
        self.queue = queue

    def run(self):
        global shutdownMavlinkTCP
        global pitchSim
        global rollSim
        global yawSim
        global xSim
        global ySim
        global zSim

        m_network = mavutil.mavlink_connection(MAVLINK_HIGHSPEED_AUTOPILOT, planner_format=False, notimestamps=True, robust_parsing=True)

	    #ToDo: Add mavlink connection error checking here
        print "Opened Mavlink high speed TCP link"

        while not shutdownMavlinkTCP:
            #recieve mavlink messages from autopilot
            mesg_s = m_network.recv_match()

            if mesg_s is not None:
                #parse attitude message
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_ATTITUDE):
                    pitchSim = mesg_s.pitch
                    rollSim = mesg_s.roll
                    yawSim = mesg_s.yaw
                    # print ("roll: ", roll, "ptch: ", pitch)
                #parse posiiton message
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
                    xSim = mesg_s.x 
                    ySim = mesg_s.y 
                    zSim = mesg_s.z

        m_network.close()        
        print("Shutting down Mavlink high speed TCP link")

class MavlinkCommandAndControlUDP(Thread):

    def __init__(self, queue):
        Thread.__init__(self)
        self.queue = queue

    def run(self):
        global shutdownMavlinkUDP
        global hud_airspeed
        global hud_altitude
        global pitch
        global roll
        global yaw
        global global_lat
        global global_lon
        global global_alt
        global global_relative_alt
        global home_lat
        global home_lon
        global home_alt
        global mode
        global waypoints
        global waypoint_quantity
        global UseSimulator

	    #Init Mavlink source
        if UseSimulator:    
            #Use this for simulation (connect to autopilot via network)	
            m_serial=mavutil.mavlink_connection(MAVLINK_C2_AUTOPILOT, planner_format=False, notimestamps=True, robust_parsing=True)
        else:
            #Use this for actual flight (connect to autopilot via serial)
            #On Nvidia Nano /dev/ttyTHS1 is J41, the largest pinheader: Tx=pin8, Rx=pin10
            m_serial = mavutil.mavlink_connection('/dev/ttyTHS1',baud=115200, planner_format=False,notimestamps=True,robust_parsing=True)

        #Init mavlink destination (connect to GCS via network)
        m_network = mavutil.mavlink_connection(MAVLINK_C2_GCS,planner_format=False, notimestamps=True, robust_parsing=True)

	    #TODO: Add mavlink connection error checking here
        print "Created Mavlink C2 links"

        while not shutdownMavlinkUDP:
            ##Autopilot -> Network##
            #recieve mavlink messages from autopilot
            mesg_s = m_serial.recv_match()

            if mesg_s is not None:

                #foward messages to network		
                m_network.write(mesg_s.get_msgbuf())

                #parse specific messages
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_ATTITUDE):
                    pitch = mesg_s.pitch  * 180 / 3.14
                    roll = mesg_s.roll * 180 / 3.14
                    yaw = mesg_s.yaw * 180 / 3.14
                    #print "roll: ", roll, "pitch: ", pitch, "hdg:", yaw
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_VFR_HUD): 
                    hud_airspeed = mesg_s.airspeed
                    hud_altitude = mesg_s.alt
                if (mesg_s.get_header().msgId ==  mavutil.ardupilotmega.MAVLINK_MSG_ID_HEARTBEAT):
                    if (mesg_s.type == 1):
                        mode = mesg_s.custom_mode
                if (mesg_s.get_header().msgId ==  mavutil.ardupilotmega.MAVLINK_MSG_ID_GLOBAL_POSITION_INT): 
                    global_lat = float(mesg_s.lat)/10000000 
                    global_lon = float(mesg_s.lon)/10000000 
                    global_alt = float(mesg_s.alt)/1000
                    global_relative_alt = float(mesg_s.relative_alt)/1000
                if (mesg_s.get_header().msgId ==  mavutil.ardupilotmega.MAVLINK_MSG_ID_HOME_POSITION): 
                    home_lat = float(mesg_s.latitude)/10000000 
                    home_lon = float(mesg_s.longitude)/10000000 
                    home_alt = float(mesg_s.altitude)/1000
                if (mesg_s.get_header().msgId ==  mavutil.ardupilotmega.MAVLINK_MSG_ID_MISSION_COUNT):
                    waypoints = []                      #clear the old maypoint list
                    waypoint_quantity = mesg_s.count
                    print waypoint_quantity
                if (mesg_s.get_header().msgId ==  mavutil.ardupilotmega.MAVLINK_MSG_ID_MISSION_ITEM_INT):
                    waypoints.append([mesg_s.seq, float(mesg_s.x)/10000000, float(mesg_s.y)/10000000, mesg_s.z, 0.0, 0.0])
                    print waypoints[mesg_s.seq]
                    #Waypoints - Compute range and bearing for each waypoint
                    if waypoints != []:
                        for i in range(len(waypoints)-1):
                            print pygeodesy.haversine(waypoints[i][1], waypoints[i][2], waypoints[i+1][1], waypoints[i+1][2])
                        for i in range(len(waypoints)-1):
                            print pygeodesy.bearing(waypoints[i][1], waypoints[i][2], waypoints[i+1][1], waypoints[i+1][2])

                
            ##Network -> Autopilot##
            #recieve mavlink messages from network
            mesg_d = m_network.recv_match()
            if mesg_d is not None:
                #foward to serial
                m_serial.write(mesg_d.get_msgbuf())

        m_network.close()
        m_serial.close()
        print("Shutting Mavlink UDP links")


#Start the Mavlink servicing threads
UDPLinkQueue = Queue()
UDPLinkWorker = MavlinkCommandAndControlUDP(UDPLinkQueue)
UDPLinkWorker.start()
if UseSimulator:
    TCPLinkQueue = Queue()
    TCPLinkWorker = MavlinkHighSpeedTCP(TCPLinkQueue)
    TCPLinkWorker.start()
    print "Sim Running - Press 'q' in the video window to quit"



while(True):

    if UseSimulator:
        #send pose to sim
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(xSim, ySim, zSim), airsim.to_quaternion(pitchSim, rollSim, yawSim)), True)
        #set weather on the sim
        #Note: These can also be changed on-the-fly via the weather UI in Unreal w/ the F10 key
        #client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)    #0.5 is a lot of fog
        #client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0) #100 is max rain

        #fetch image from sim
        rawImage = client.simGetImage("0", cameraType)
        if (rawImage == None):
            print("Camera is not returning image, please check airsim for error messages")
            continue    #TODO: fix this so the GCS downlink still works if no Arisim    
        else:
            #convert into a standard OpenCV image frame
            image = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)

    else: #real flight
        #fetch image from gstreamer
        ret, image = cap.read()
        #trim the frame to match the aspect ratio
        image = image[0:1080, 240:1680]
        image = cv2.resize(image, (640,480), interpolation=cv2.INTER_AREA)
        

    if (image is not None):
        #draw center marker
        cv2.rectangle(image, (310,230), (330,250), (0,255,0),2)

        #Airspeed Slider
        airspeed_vert_pos = util.lim(-int(hud_airspeed*16)+480,470,30)
        cv2.putText(image, str(round(hud_airspeed,1)), (10,airspeed_vert_pos), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

        #Altitude Slider
        #slider_display_alt = hud_altitude  #for MSL alt (m)
        slider_display_alt = global_relative_alt  #for alt above takeoff (m)
        altitude_vert_pos = util.lim(-int(slider_display_alt*1.5)+480,470,30)
        cv2.putText(image, str(int(round(slider_display_alt,0))), (580,altitude_vert_pos), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

        #Flight mode display
        cv2.putText(image, flightMode(mode), (270,20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,    (0, 255, 0), 1, cv2.LINE_AA)      

        #Horizon Line
        horizonEarthFrameLeft = numpy.array([[CAMERA_FOV],[-CAMERA_FOV],[0]])    #point x,y,z
        horizonEarthFrameRight = numpy.array([[CAMERA_FOV],[CAMERA_FOV],[0]])    #point x,y,z
        horizonCameraFrameLeft = util.rotateEarth2Camera(horizonEarthFrameLeft,roll,pitch,0)
        horizonCameraFrameRight = util.rotateEarth2Camera(horizonEarthFrameRight,roll,pitch,0)
        horizonOpenCVFrameLeft = util.convertCamera2OpenCV(horizonCameraFrameLeft)
        horizonOpenCVFrameRight = util.convertCamera2OpenCV(horizonCameraFrameRight)
        cv2.line(image, ( int(horizonOpenCVFrameLeft[0]),int(horizonOpenCVFrameLeft[1]) ),
                    ( int(horizonOpenCVFrameRight[0]),int(horizonOpenCVFrameRight[1]) ),(0,255,0),2)

        ##EastTic
        #a = math.radians(20)
        #horizonEarthFrameLeft = numpy.array([[CAMERA_FOV],[0],[30]])    #Draw vertical tic
        #horizonEarthFrameRight = numpy.array([[CAMERA_FOV],[0],[0]])
        ##Rotate it facing east (azmuth) at 10deg below the horizon (elevation)  
        #horizonEarthFrameLeft = rotateCamera2Earth(horizonEarthFrameLeft, 0,-10,90)
        #horizonEarthFrameRight = rotateCamera2Earth(horizonEarthFrameRight, 0,-10,90)  
        ##Rotate it based on vehicle attitude
        #horizonCameraFrameLeft = rotateEarth2Camera(horizonEarthFrameLeft,roll,pitch,heading)
        #horizonCameraFrameRight = rotateEarth2Camera(horizonEarthFrameRight,roll,pitch,heading)
        #horizonOpenCVFrameLeft = convertCamera2OpenCV(horizonCameraFrameLeft)
        #horizonOpenCVFrameRight = convertCamera2OpenCV(horizonCameraFrameRight)
        #cv2.line(frame, ( int(horizonOpenCVFrameLeft[0]),int(horizonOpenCVFrameLeft[1]) ),                    ( int(horizonOpenCVFrameRight[0]),int(horizonOpenCVFrameRight[1]) ),(0,255,0),2)
        

        #display FPS 
        #cv2.putText(image,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,0,255),thickness)

        #TODO: only if sim
        #display locally
        cv2.imshow("Video", image)

        #send to gstreamer
        image = image[:,:,0:3]  #Resize from 4 value pixels to 3 value pixels
        out.write(image)	    

        #calculate frame rate
        #frameCount  = frameCount  + 1
        #endTime=time.clock()
        #diff = endTime - startTime
        #if (diff > 1):
        #    fps = frameCount
        #    frameCount = 0
        #    startTime = endTime

        #TODO: only if sim    
        key = cv2.waitKey(1) & 0xFF
        if (key == 27 or key == ord('q') or key == ord('x')):
            shutdownMavlinkUDP = True
            shutdownMavlinkTCP = True
            break

#Shutdown
UDPLinkQueue.join()
TCPLinkQueue.join()
if not UseSimulator:
    cap.release()
out.release()
cv2.destroyAllWindows()



