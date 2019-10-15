
cd /home/greywing/Projects/CSCAPE

#modify serial port privlidges
echo admin | sudo -S chmod +r /dev/ttyTHS1

#run serial to UDP
python2 udpC2.py &

#H.265 stream
#gst-launch-1.0 v4l2src device=/dev/video0 ! 'video/x-raw, format=(string)YUY2, width=(int)640, height=(int)480, framerate=(fraction)30/1' ! videoconvert ! 'video/x-raw, format=(string)I420'  ! videorate ! 'video/x-raw, framerate=(fraction)7/1' ! omxh265enc control-rate=2 bitrate=100000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! rtph265pay mtu=1400 ! udpsink host=192.168.10.21 port=5000 sync=false async=false &

#H.265 stream with resizing
gst-launch-1.0 v4l2src device=/dev/video0 ! 'video/x-raw, format=(string)YUY2, width=(int)1280, height=(int)720, framerate=(fraction)10/1' ! videoconvert ! 'video/x-raw, format=(string)I420'  ! videorate ! 'video/x-raw, framerate=(fraction)5/1' ! videocrop left=160 right=160 ! videoscale ! 'video/x-raw, width=(int)640, height=(int)480' ! omxh265enc control-rate=2 bitrate=200000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! rtph265pay mtu=1400 ! udpsink host=192.168.10.21 port=50006 sync=false async=false &

#H.265 zoomed
#gst-launch-1.0 v4l2src device=/dev/video0 ! 'video/x-raw, format=(string)YUY2, width=(int)1280, height=(int)720, framerate=(fraction)10/1' ! videoconvert ! 'video/x-raw, format=(string)I420'  ! videorate ! 'video/x-raw, framerate=(fraction)5/1' ! videocrop top=120 left=320 right=320 bottom=120 ! omxh265enc control-rate=2 bitrate=100000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! rtph265pay mtu=1400 ! udpsink host=192.168.10.21 port=50008 sync=false async=false &
