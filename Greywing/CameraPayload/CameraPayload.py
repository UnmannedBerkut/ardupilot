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

import pprint
import os
import time
from pymavlink import mavutil
import numpy as np
import sys
import cv2
from threading import Thread
from queue import Queue

#pymavlink data
airspeed = 0.0    #pitot static airspeed (m/s)
altitude = 0.0    #baro altitude above reference(above takeoff) (m)
yaw = 0           #IMU heading (magnetometer) (deg true)
pitch = 0.0       #(deg)
roll = 0.0        #(deg)
x = 0
y = 0
z = 0 
mode = 0
waypoints = []        #list of tuples containing known waypoints
waypoint_quantity = 0 #the number of known waypoints as reported by the autopilot, if this is != len(waypoints) that means some waypoints are missing
FOV = 1000             #camera field of view (determined experimentally)
shutdown = False

#cameraType = "infrared"
cameraType = "scene"

for arg in sys.argv[1:]:
  cameraType = arg.lower()

cameraTypeMap = { 
 "depth": airsim.ImageType.DepthVis,
 "segmentation": airsim.ImageType.Segmentation,
 "seg": airsim.ImageType.Segmentation,
 "scene": airsim.ImageType.Scene,
 "disparity": airsim.ImageType.DisparityNormalized,
 "normals": airsim.ImageType.SurfaceNormals,
 "infrared": airsim.ImageType.Infrared
}

fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
thickness = 2
textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
print (textSize)
textOrg = (10, 10 + textSize[1])
frameCount = 0
startTime=time.clock()
fps = 0

pp = pprint.PrettyPrinter(indent=4)

client = airsim.VehicleClient('192.168.10.21')
print("Waiting for AirSim Server") 
client.confirmConnection()
print("Connected to Airsim Server") 
client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0)

#Add SR1 rate setting to veicle settings
#Note: TCP 5762 = SR1 in settings
# Use Mission planner to set in the meantime
# Set SR1 rates to be SR1_EXTRA1 = SR1_POSITION = 20
# SR1_PARAMS = 10 (not entirely necessary)
# Set other SR1 rates to zero

#Note: Errors in gstreamer pipe may silentley fail (escpically on Linux).
#Check pipes using the command line first
#out = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw, format=(string)I420  ! videorate ! video/x-raw, framerate=(fraction)5/1 ! omxh264enc control-rate=2 bitrate=400000 ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay mtu=1400 ! udpsink host=192.168.10.21 port=50006 sync=false async=false', cv2.CAP_GSTREAMER,0,5,(640,480), True)
out = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw, format=(string)I420  ! videorate ! video/x-raw, framerate=(fraction)5/1 ! x264enc bitrate=300 tune=zerolatency ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay mtu=1400 ! udpsink host=192.168.10.21 port=50006 sync=false async=false', cv2.CAP_GSTREAMER,0,5,(640,480), True)

if (out.isOpened()):
    print("Pipe Created Sucessfully")
else:
    print("Pipe Creation Failed")

class MavlinkWorker(Thread):

    def __init__(self, queue):
        Thread.__init__(self)
        self.queue = queue

    def run(self):
        global shutdown
        global airspeed
        global altitude
        global pitch
        global roll
        global yaw
        global x
        global y
        global z
        global mode
        global waypoints
        global waypoint_quantity

        m_network = mavutil.mavlink_connection('tcp:192.168.10.10:5762', planner_format=False, notimestamps=True, robust_parsing=True)

	#ToDo: Add mavlink connection error checking here

        while not shutdown:
            #recieve mavlink messages from autopilot
            mesg_s = m_network.recv_match()

            if mesg_s is not None:
                #foward messages to network
                #m_network.write(mesg_s.get_msgbuf())

                #parse specific messages
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_ATTITUDE):
                    pitch = mesg_s.pitch
                    roll = mesg_s.roll
                    yaw = mesg_s.yaw
                    # print ("roll: ", roll, "ptch: ", pitch)
                if (mesg_s.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
                    x = mesg_s.x 
                    y = mesg_s.y 
                    z = mesg_s.z
                    #print ("x: ", x, "y: ", y)
                #if (mesg_s.get_header.im_class == mavutil.ardupilotmega.MAVLink_vfr_hud_message):
                #    airspeed = mesg_s.airspeed
                #    altitude = mesg_s.alt
                #    heading =  mesg_s.heading
                #if (mesg_s.get_header.im_class ==  mavutil.ardupilotmega.MAVLink_heartbeat_message):
                #    mode = mesg_s.custom_mode
                #if (mesg_s.get_header.im_class ==  mavutil.ardupilotmega.MAVLink_mission_count_message):
                #    waypoints = []                      #clear the old maypoint list
                #    waypoint_quantity = mesg_s.count
                #    print(waypoint_quantity)
                #if (mesg_s.get_header.im_class ==  mavutil.ardupilotmega.MAVLink_mission_item_int_message):
                #    waypoints.append([mesg_s.seq, float(mesg_s.x)/10000000, float(mesg_s.y)/10000000, mesg_s.z, 0.0, 0.0])
                #    print(waypoints[mesg_s.seq])


            #recieve mavlink messages from network
            #mesg_d = m_network.recv_match()
            #if mesg_d is not None:
            #    #foward to serial
            #    m_serial.write(mesg_d.get_msgbuf())

        print("shutting down mavlink")


#Start mavlink read thread
queue = Queue()
worker = MavlinkWorker(queue)
#worker.daemon = True    #Setting daemon to True will let the main thread exit even though the workers are blocking
worker.start()

while(True):
    #m_msg = m_network.recv_match()

    #if m_msg is not None:

    #    if (m_msg.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_ATTITUDE):
    #        pitch = m_msg.pitch #  * 180 / 3.14
    #        roll = m_msg.roll # * 180 / 3.14
    #        yaw = m_msg.yaw # * 180 / 3.14
    #        #print(yaw * 180/3.14)

    #    if (m_msg.get_header().msgId == mavutil.ardupilotmega.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
    #        x = m_msg.x 
    #        y = m_msg.y 
    #        z = m_msg.z 
    #        #print(x, y, z)


    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(x, y, z), airsim.to_quaternion(pitch, roll, yaw)), True)


    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    rawImage = client.simGetImage("0", cameraTypeMap[cameraType])
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        if (png is not None):
            cv2.putText(png,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,0,255),thickness)
            cv2.imshow("Depth", png)
            png = png[:,:,0:3]  #Resize from 4 value pixels to 3 value pixels
            out.write(png)


    frameCount  = frameCount  + 1
    endTime=time.clock()
    diff = endTime - startTime
    if (diff > 1):
        fps = frameCount
        frameCount = 0
        startTime = endTime
    
    key = cv2.waitKey(1) & 0xFF
    if (key == 27 or key == ord('q') or key == ord('x')):
        shutdown = True
        break

