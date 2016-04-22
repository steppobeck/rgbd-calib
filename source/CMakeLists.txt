add_executable(measure_pose_offset measure_pose_offset.cpp)
target_link_libraries(measure_pose_offset framework ${LIBRARIES})

add_executable(zmq_sender zmq_sender.cpp)
target_link_libraries(zmq_sender framework ${LIBRARIES})

add_executable(zmq_receiver zmq_receiver.cpp)
target_link_libraries(zmq_receiver framework ${LIBRARIES})

add_executable(viewer viewer.cpp)
target_link_libraries(viewer framework ${LIBRARIES})

add_executable(tracker tracker.cpp)
target_link_libraries(tracker framework ${LIBRARIES})

add_executable(initialize initialize.cpp)
target_link_libraries(initialize framework ${LIBRARIES})

add_executable(stable_sampler stable_sampler.cpp)
target_link_libraries(stable_sampler framework ${LIBRARIES})

add_executable(calibrate_intrinsic calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic framework ${LIBRARIES})

add_executable(sweep_sampler sweep_sampler.cpp)
target_link_libraries(sweep_sampler framework ${LIBRARIES})

add_executable(display_sweep display_sweep.cpp)
target_link_libraries(display_sweep framework ${LIBRARIES})

add_executable(test_stuff test_stuff.cpp)
target_link_libraries(test_stuff framework ${LIBRARIES})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate framework ${LIBRARIES})

add_executable(sweep_calibrate sweep_calibrate.cpp)
target_link_libraries(sweep_calibrate framework ${LIBRARIES})

add_executable(evaluate evaluate.cpp)
target_link_libraries(evaluate framework ${LIBRARIES})

add_executable(record record.cpp)
target_link_libraries(record framework ${LIBRARIES})

add_executable(play play.cpp)
target_link_libraries(play framework ${LIBRARIES})

add_executable(play_sweep play_sweep.cpp)
target_link_libraries(play_sweep framework ${LIBRARIES})

add_executable(convert_recording_to_point_cloud convert_recording_to_point_cloud.cpp)
target_link_libraries(convert_recording_to_point_cloud framework ${LIBRARIES})