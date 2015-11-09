add_executable(viewer viewer.cpp)
target_link_libraries(viewer framework ${LIBRARIES})
add_dependencies(viewer glfw framework ${COPY_BINARY})

add_executable(initialize initialize.cpp)
target_link_libraries(initialize framework ${LIBRARIES})
add_dependencies(initialize glfw framework ${COPY_BINARY})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate framework ${LIBRARIES})
add_dependencies(calibrate glfw framework ${COPY_BINARY})

add_executable(record record.cpp)
target_link_libraries(record framework ${LIBRARIES})
add_dependencies(record glfw framework ${COPY_BINARY})

add_executable(play play.cpp)
target_link_libraries(play framework ${LIBRARIES})
add_dependencies(play glfw framework ${COPY_BINARY})


