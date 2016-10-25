add_executable(measure_pose_offset measure_pose_offset.cpp)
target_link_libraries(measure_pose_offset framework)
install(TARGETS measure_pose_offset DESTINATION bin)

add_executable(zmq_sender zmq_sender.cpp)
target_link_libraries(zmq_sender framework)
install(TARGETS zmq_sender DESTINATION bin)

add_executable(zmq_receiver zmq_receiver.cpp)
target_link_libraries(zmq_receiver framework)
install(TARGETS zmq_receiver DESTINATION bin)

add_executable(viewer viewer.cpp)
target_link_libraries(viewer framework)
install(TARGETS viewer DESTINATION bin)

add_executable(tracker tracker.cpp)
target_link_libraries(tracker framework)
install(TARGETS tracker DESTINATION bin)

add_executable(initialize initialize.cpp)
target_link_libraries(initialize framework)
install(TARGETS initialize DESTINATION bin)

add_executable(stable_sampler stable_sampler.cpp)
target_link_libraries(stable_sampler framework)
install(TARGETS stable_sampler DESTINATION bin)

add_executable(rgbdir_viewer rgbdir_viewer.cpp)
target_link_libraries(rgbdir_viewer framework)
install(TARGETS rgbdir_viewer DESTINATION bin)

add_executable(calibrate_intrinsic calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic framework)
install(TARGETS calibrate_intrinsic DESTINATION bin)

add_executable(sweep_sampler sweep_sampler.cpp)
target_link_libraries(sweep_sampler framework)
install(TARGETS sweep_sampler DESTINATION bin)

add_executable(display_sweep display_sweep.cpp)
target_link_libraries(display_sweep framework)
install(TARGETS display_sweep DESTINATION bin)

add_executable(test_stuff test_stuff.cpp)
target_link_libraries(test_stuff framework)
install(TARGETS test_stuff DESTINATION bin)

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate framework)
install(TARGETS calibrate DESTINATION bin)

add_executable(sweep_calibrate sweep_calibrate.cpp)
target_link_libraries(sweep_calibrate framework)
install(TARGETS sweep_calibrate DESTINATION bin)

add_executable(evaluate evaluate.cpp)
target_link_libraries(evaluate framework)
install(TARGETS evaluate DESTINATION bin)

add_executable(split_samples split_samples.cpp)
target_link_libraries(split_samples framework)
install(TARGETS split_samples DESTINATION bin)

add_executable(record record.cpp)
target_link_libraries(record framework)
install(TARGETS record DESTINATION bin)

add_executable(play play.cpp)
target_link_libraries(play framework)
install(TARGETS play DESTINATION bin)

add_executable(play_sweep play_sweep.cpp)
target_link_libraries(play_sweep framework)
install(TARGETS play_sweep DESTINATION bin)

add_executable(sweep_stats sweep_stats.cpp)
target_link_libraries(sweep_stats framework)
install(TARGETS sweep_stats DESTINATION bin)

add_executable(convert_recording_to_point_cloud convert_recording_to_point_cloud.cpp)
target_link_libraries(convert_recording_to_point_cloud framework)
install(TARGETS convert_recording_to_point_cloud DESTINATION bin)

add_executable(send_recording_as_point_cloud send_recording_as_point_cloud.cpp)
target_link_libraries(send_recording_as_point_cloud framework)
install(TARGETS send_recording_as_point_cloud DESTINATION bin)


#install(DIRECTORY . DESTINATION bin FILES_MATCHING PATTERN "poseoffset")
#install(DIRECTORY . DESTINATION bin FILES_MATCHING PATTERN "click_x.wav")

