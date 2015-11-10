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

# Working examples from github
./initialize ../../../data/30.cv ../../../data/30_init
(./record ../../../data/30.stream 141.54.147.27:7000)
./play ../../../data/30.stream 141.54.147.27:7000
./viewer ../../../data/30.cv 141.54.147.27:7000



# history of some usefull commands:

+ initialize 
env DISPLAY=:0.0 ./Protonect 179625534347 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/23_init
./initialize ../../../data/23.cv ../../../data/23_init
+ calibrate
env DISPLAY=:0.0 ./Protonect 179625534347 -s 141.54.147.32:7000 -a 5000 -n
./calibrate ../../../data/23.cv 141.54.147.32:7000
+ view calibrated stream
./viewer ../../../data/23.cv 141.54.147.30:7000


# device serials
23 - 179625534347
24 - 110356534447
25 - 007688634347
26 - 084864534447
27 - 049678134347

50 - 011312650647
51 -
52 -
53 -
54 -


# device serials with option -r
30 - 505545442542
40 - 505527442542
41 - 501411241942
42 - 505573342542




