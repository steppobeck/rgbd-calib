add_executable(measure_pose_offset measure_pose_offset.cpp)
target_link_libraries(measure_pose_offset framework ${LIBRARIES})
add_dependencies(measure_pose_offset glfw framework ${COPY_BINARY})

add_executable(viewer viewer.cpp)
target_link_libraries(viewer framework ${LIBRARIES})
add_dependencies(viewer glfw framework ${COPY_BINARY})

add_executable(tracker tracker.cpp)
target_link_libraries(tracker framework ${LIBRARIES})
add_dependencies(tracker glfw framework ${COPY_BINARY})

add_executable(initialize initialize.cpp)
target_link_libraries(initialize framework ${LIBRARIES})
add_dependencies(initialize glfw framework ${COPY_BINARY})

add_executable(stable_sampler stable_sampler.cpp)
target_link_libraries(stable_sampler framework ${LIBRARIES})
add_dependencies(stable_sampler glfw framework ${COPY_BINARY})

add_executable(sweep_sampler sweep_sampler.cpp)
target_link_libraries(sweep_sampler framework ${LIBRARIES})
add_dependencies(sweep_sampler glfw framework ${COPY_BINARY})

add_executable(display_sweep display_sweep.cpp)
target_link_libraries(display_sweep framework ${LIBRARIES})
add_dependencies(display_sweep glfw framework ${COPY_BINARY})

add_executable(test_stuff test_stuff.cpp)
target_link_libraries(test_stuff framework ${LIBRARIES})
add_dependencies(test_stuff glfw framework ${COPY_BINARY})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate framework ${LIBRARIES})
add_dependencies(calibrate glfw framework ${COPY_BINARY})

add_executable(evaluate evaluate.cpp)
target_link_libraries(evaluate framework ${LIBRARIES})
add_dependencies(evaluate glfw framework ${COPY_BINARY})

add_executable(record record.cpp)
target_link_libraries(record framework ${LIBRARIES})
add_dependencies(record glfw framework ${COPY_BINARY})

add_executable(play play.cpp)
target_link_libraries(play framework ${LIBRARIES})
add_dependencies(play glfw framework ${COPY_BINARY})

add_executable(play_sweep play_sweep.cpp)
target_link_libraries(play_sweep framework ${LIBRARIES})
add_dependencies(play_sweep glfw framework ${COPY_BINARY})


add_executable(convert_recording_to_point_cloud convert_recording_to_point_cloud.cpp)
target_link_libraries(convert_recording_to_point_cloud framework ${LIBRARIES})
add_dependencies(convert_recording_to_point_cloud glfw framework ${COPY_BINARY})

