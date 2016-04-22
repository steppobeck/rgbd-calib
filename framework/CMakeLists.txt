file(GLOB FRAMEWORK_SOURCE *.cpp)
file(GLOB FRAMEWORK_INLINE *.inl)
file(GLOB FRAMEWORK_HEADER *.hpp)

source_group("Inline Files" FILES ${FRAMEWORK_INLINE})

add_library(framework STATIC glew.c
  ${FRAMEWORK_SOURCE}
  ${FRAMEWORK_INLINE}
  ${FRAMEWORK_HEADER})

target_include_directories(framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})