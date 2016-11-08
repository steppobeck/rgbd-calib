initialize
+ 

stable_sampler
+ 

calibrate
+ 

viewer
+

record
+ freely configurable recorder based on zmq
+ receives frames of multiple Kinect 2.0 from serverport
+ writes frames to disk

play
+ freely configurable player based on zmq
+ reads frames of multiple Kinect 2.0 from disk
+ streams frames to serverport


# device serials of Kinect V2 sensors in our lab:
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

# IPs of Kinect-servers and attached Kinects in our lab:
+ boreas 141.54.147.32 with Kinect 50 and 51 and 54
+ arachne 141.54.147.27 with Kinect 52 and 53
+ charon 141.54.147.33 with Kinect 23, 24, 25, 26, 27



# Example calibration for Kinect 50 on boreas
0. update the correction offset of the checkerboard target (optional)
./measure_pose_offset
1. init the calibration:
./protonect.sh 011312650647 -a 5000 -s 141.54.147.32:7000 -y 50 ~/Desktop/my-git/rgbd-calib/data/50_init 6
rm ../../../data/50.cv_*
./initialize ../../../data/50.cv ../../../data/50_init

2. sampling using checkerboard at 5 locations:
./protonect.sh 011312650647 -s 141.54.147.32:7000 -n -i
./stable_sampler ../../../data/50.cv 141.54.147.32:7000 -s 5

3. calibrate:
./calibrate ../../../data/50.cv

# To add further samples to the exisiting calibration do:
1. re-init the calibration:
./initialize ../../../data/50.cv ../../../data/50_init

2. Add further samples by sampling using checkerboard at e.g. 5 additional locations:
./protonect.sh 011312650647 -s 141.54.147.32:7000 -n -i
./stable_sampler ../../../data/50.cv 141.54.147.32:7000 -s 5

3. calibrate:
./calibrate ../../../data/50.cv




# Quickguide for Marcel
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

