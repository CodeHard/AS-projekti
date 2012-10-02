FILE (GLOB_RECURSE project_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (project_MOC_HEADERS app.h)
SET (project_LIBS ${Boost_LIBRARIES} ${PCL_LIBRARIES} /usr/lib/libOpenNI.so)
SET (project_BIN ${PROJECT_NAME})

ADD_EXECUTABLE(object_recognizer ObjectRecognizer.cc)
TARGET_LINK_LIBRARIES(object_recognizer ${project_LIBS})
SET_TARGET_PROPERTIES(object_recognizer PROPERTIES VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}" OUTPUT_NAME object_recognizer CLEAN_DIRECT_OUTPUT 1)

INSTALL(TARGETS object_recognizer DESTINATION bin)
