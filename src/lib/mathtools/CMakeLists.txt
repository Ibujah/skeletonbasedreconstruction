set(LIBRARY_NAME ${MATHTOOLS_LIB})

set(SOURCE_FILES    application/BsplineUtils.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)

