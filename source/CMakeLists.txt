add_executable(example example.cpp)
target_link_libraries(example framework ${LIBRARIES})
add_dependencies(example glfw framework ${COPY_BINARY})


add_executable(tests tests.cpp)
target_link_libraries(tests framework ${LIBRARIES})


