set(LIBRARY_NAME ${UTILS_LIB})

include_directories(${CMAKE_SOURCE_DIR}/src/lib)
set(SOURCE_FILES    DrawShape.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

target_link_libraries(${LIBRARY_NAME} ${SHAPE_LIB}
									  ${OpenCV_LIBS})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)

