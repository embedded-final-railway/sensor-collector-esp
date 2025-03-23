FILE(GLOB_RECURSE app_sources *.h *.cpp)

idf_component_register(SRCS ${app_sources} INCLUDE_DIRS ".")
