import socket
#import serial
from pymavlink import mavutil
import time

UDP_IP = "192.168.10.255"
UDP_PORT = 5005
SEND_BROADCAST = False

if (SEND_BROADCAST):
    #Init UDP socket - for broadcast output
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)    

#init mavlink source (serial)
#On Nvidia Nano /dev/ttyTHS1 is J41, the largest pinheader: Tx=pin8, Rx=pin10
m_serial = mavutil.mavlink_connection('/dev/ttyTHS1', baud=115200, planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)

#init mavlink destination (network)
m_network = mavutil.mavlink_connection('udpout:192.168.10.21:50005',planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)

while True:

    #recieve from autopilot -> foward to network
    mesg_s = m_serial.recv_match();
    if mesg_s is not None:
        m_network.write(mesg_s.get_msgbuf())
        if (SEND_BROADCAST):
            sock.sendto(mesg_s.get_msgbuf(), (UDP_IP, UDP_PORT))

        #Example: Print AHRS roll value
        #if (mesg_s.get_header.im_class == mavutil.ardupilotmega.MAVLink_attitude_message):
        #    print "roll: ", mesg_s.roll * 180 / 3.14


    #recieve from network -> foward to autopilot
    mesg_d = m_network.recv_match();
    if mesg_d is not None:
        m_serial.write(mesg_d.get_msgbuf())



