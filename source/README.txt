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
env DISPLAY=:0.0 ./Protonect 179625534347 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/23_init 6
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
51 - 011482550647
52 - 012086450647
53 - 016215650647
54 - 012126250647


# device serials with option -r
30 - 505545442542
40 - 505527442542
41 - 501411241942
42 - 505573342542



# Boreas (IP is 141.54.147.32) with Kinect 50 and 51

-----------------> 50
init:
./protonect.sh 011312650647 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/50_init 6
./initialize ../../../data/50.cv ../../../data/50_init
calibrate:
./protonect.sh 011312650647 -s 141.54.147.32:7000 -n -i
./calibrate ../../../data/50.cv 141.54.147.32:7000 -s 5
play:
./protonect.sh 011312650647 -s 141.54.147.32:7000 -n
./viewer ../../../data/50.cv 141.54.147.32:7000


-----------------> 51
init:
./protonect.sh 011482550647 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/51_init 6
./initialize ../../../data/51.cv ../../../data/51_init
calibrate:
./protonect.sh 011482550647 -s 141.54.147.32:7000 -n -i
./calibrate ../../../data/51.cv 141.54.147.32:7000 -s 5
play:
./protonect.sh 011482550647 -s 141.54.147.32:7000 -n
./viewer ../../../data/51.cv 141.54.147.32:7000

-----------------> 50 and 51
./protonect.sh 011312650647 011482550647 -s 141.54.147.32:7000 -n
./viewer ../../../data/50.cv ../../../data/51.cv 141.54.147.32:7000

# in Avango
./protonect.sh 011312650647 011482550647 -s 141.54.147.32:7000
cd /opt/avango/new_renderer/examples/video3d
zsh
./start.sh ~/Desktop/my-git/rgbd-calib/data/surface_50_51.ks

-----------------> 54
init:
./protonect.sh 012126250647 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/54_init 6
./initialize ../../../data/54.cv ../../../data/54_init
calibrate:
./protonect.sh 012126250647 -s 141.54.147.32:7000 -n -i
./calibrate ../../../data/54.cv 141.54.147.32:7000 -s 5
play:
./protonect.sh 012126250647 -s 141.54.147.32:7000 -n
./viewer ../../../data/54.cv 141.54.147.32:7000



# Arachne (IP is 141.54.147.27) with Kinect 52 and 53
-----------------> 52
init:
./protonect.sh 012086450647 -a 5000 -s 141.54.147.27:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/52_init 6
./initialize ../../../data/52.cv ../../../data/52_init
calibrate:
./protonect.sh 012086450647 -s 141.54.147.27:7000 -n -i
./calibrate ../../../data/52.cv 141.54.147.27:7000 -s 5
play:
./protonect.sh 012086450647 -s 141.54.147.27:7000 -n
./viewer ../../../data/52.cv 141.54.147.27:7000

-----------------> 53
init:
./protonect.sh 016215650647 -a 5000 -s 141.54.147.27:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/53_init 6
./initialize ../../../data/53.cv ../../../data/53_init
calibrate:
./protonect.sh 016215650647 -s 141.54.147.27:7000 -n -i
./calibrate ../../../data/53.cv 141.54.147.27:7000 -s 5
play:
./protonect.sh 016215650647 -s 141.54.147.27:7000 -n
./viewer ../../../data/53.cv 141.54.147.27:7000


# Achill 23 24 25 36
-----------------> 23
init:
./protonect.sh 179625534347 -a 5000 -s 141.54.147.33:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/23_init 6
./initialize ../../../data/23.cv ../../../data/23_init
calibrate:
./protonect.sh 179625534347 -s 141.54.147.33:7000 -n -i
./calibrate ../../../data/23.cv 141.54.147.33:7000 -s 5
play:
./protonect.sh 179625534347 -s 141.54.147.33:7000 -n
./viewer ../../../data/23.cv 141.54.147.33:7000

-----------------> 24
init:
./protonect.sh 110356534447 -a 5000 -s 141.54.147.33:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/24_init 6
./initialize ../../../data/24.cv ../../../data/24_init
calibrate:
./protonect.sh 110356534447 -s 141.54.147.33:7000 -n -i
./calibrate ../../../data/24.cv 141.54.147.33:7000 -s 5
play:
./protonect.sh 110356534447 -s 141.54.147.33:7000 -n
./viewer ../../../data/24.cv 141.54.147.33:7000

-----------------> 25
init:
./protonect.sh 007688634347 -a 5000 -s 141.54.147.33:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/25_init 6
./initialize ../../../data/25.cv ../../../data/25_init
calibrate:
./protonect.sh 007688634347 -s 141.54.147.33:7000 -n -i
./calibrate ../../../data/25.cv 141.54.147.33:7000 -s 5
play:
./protonect.sh 007688634347 -s 141.54.147.33:7000 -n
./viewer ../../../data/25.cv 141.54.147.33:7000

-----------------> 26
init:
./protonect.sh 084864534447 -a 5000 -s 141.54.147.33:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/26_init 6
./initialize ../../../data/26.cv ../../../data/26_init
calibrate:
./protonect.sh 084864534447 -s 141.54.147.33:7000 -n -i
./calibrate ../../../data/26.cv 141.54.147.33:7000 -s 5
play:
./protonect.sh 084864534447 -s 141.54.147.33:7000 -n
./viewer ../../../data/26.cv 141.54.147.33:7000




# For Marcel

Trouble shooting:
- If Kinect server hangs,
killall Protonect -KILL


A. record from server charon with 4 Kinects attached

cd /opt/kinect-resources
- WITH compressed color
./protonect.sh 179625534347 110356534447 007688634347 084864534447 -s 141.54.147.33:7000
./record /media/usbplatte/kinect_shots/23242526_compressed.stream 141.54.147.33:7000 -c -n 250 -k 4 -w 10

A2 Live view
./kinect_client kinect_surface_K_23_24_25_26.ksV3

- stream from server and record WITHOUT compression
./protonect.sh 179625534347 110356534447 007688634347 084864534447 -s 141.54.147.33:7000 -n
./record /media/usbplatte/kinect_shots/23242526_without_compression.stream 141.54.147.33:7000 -n 250 -k 4 -w 10
A3 Live view
./viewer ../../../data/23.cv ../../../data/24.cv ../../../data/25.cv ../../../data/26.cv 141.54.147.33:7000



B. play a recording for live view
- with compression
./play /media/usbplatte/kinect_shots/23242526_compressed.stream 141.54.147.33:7000 -f 20 -k 4 -c
./kinect_client kinect_surface_K_23_24_25_26.ksV3

- without compression
./play /media/usbplatte/kinect_shots/23242526_without_compression.stream 141.54.147.33:7000 -f 20 -k 4
./viewer ../../../data/23.cv ../../../data/24.cv ../../../data/25.cv ../../../data/26.cv 141.54.147.33:7000

# Quickguide
Server:
cd /opt/kinect-resources
./protonect.sh 179625534347 110356534447 007688634347 084864534447 -s 141.54.147.33:7000

Live:
cd /home/steppo/Desktop/my-svn/multiViewTools/libKinect/examples
./kinect_client kinect_surface_K_23_24_25_26.ksV3

Record:
/home/karnapke/Desktop/my-git/rgbd-calib/build/build/Release
./record 23242526_compressed.stream 141.54.147.33:7000 -n 250 -k 4 -w 100 -c

Check:
/home/karnapke/Desktop/my-git/rgbd-calib/build/build/Release
./play 23242526_compressed.stream 141.54.147.33:7000 -f 20 -k 4 -c

/home/steppo/Desktop/my-svn/multiViewTools/libKinect/examples
./kinect_client kinect_surface_K_23_24_25_26.ksV3

