set(LIBRARY_NAME ${FILEIO_LIB})

include_directories(${CMAKE_SOURCE_DIR}/src/lib)
set(SOURCE_FILES   CameraFile.cpp
				   BoundaryFile.cpp
				   RecSkelFile.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)

