set(LIBRARY_NAME ${DISPLAY3D_LIB})

set(SOURCE_FILES DisplayClass.cpp
				 DisplayFrame.cpp
				 DisplayBoundary.cpp
				 DisplayCamera.cpp
				 DisplaySkeleton.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

include_directories(${CMAKE_SOURCE_DIR}/src/lib/
					${SFML_INCLUDE_DIR}
					${Boost_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} ${MATHTOOLS_LIB}
									  ${CAMERA_LIB}
									  ${OPENGL_LIBRARIES}
									  ${SFML_LIBRARIES})


SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
