FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/*.h ${CMAKE_SOURCE_DIR}/*.cpp)

idf_component_register(SRCS ${app_sources})