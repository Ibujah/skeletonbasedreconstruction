set(LIBRARY_NAME ${TRACKING_LIB})

set(SOURCE_FILES Tracker.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

include_directories(${CMAKE_SOURCE_DIR}/src/lib/
					${ARTOOLKIT5_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} ${MATHTOOLS_LIB}
									  ${CAMERA_LIB}
									  ${ARTOOLKIT5_LIBRARIES})


SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
