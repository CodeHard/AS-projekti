FILE (GLOB_RECURSE project_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (project_MOC_HEADERS app.h)
SET (project_LIBS ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OPENNI_LIBRARIES})
SET (project_BIN ${PROJECT_NAME})

ADD_EXECUTABLE(object_scanner ObjectScanner.cc)
TARGET_LINK_LIBRARIES(object_scanner ${project_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
SET_TARGET_PROPERTIES(object_scanner PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME object_scanner CLEAN_DIRECT_OUTPUT 1)

INSTALL(TARGETS object_scanner DESTINATION bin)
