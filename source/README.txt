initialize
+ 

calibrate
+ 

viewer
+ supports currently only one Kinect 2.0 sensor


record
+ freely configurable recorder based on zmq
+ receives frames of multiple Kinect 2.0 from serverport
+ writes frames to disk

play
+ freely configurable player based on zmq
+ reads frames of multiple Kinect 2.0 from disk
+ streams frames to serverport

EXAMPLES:

1. stream a single Kinect 2.0 using Protonect and viewer




# history of some usefull commands:
sudo env DISPLAY=:0.0 ./Protonect 505545442542 -a 5000 -s 141.54.147.27:7000 -r -n
./record kinect_30.stream 141.54.147.27:7000
./play kinect_30.stream 141.54.147.27:7000
./viewer /home/steppo/Desktop/my-svn/multiViewTools/libKinect/examples/rgbd_calibV2/kinect_23_calibration.cv 141.54.147.27:7000

