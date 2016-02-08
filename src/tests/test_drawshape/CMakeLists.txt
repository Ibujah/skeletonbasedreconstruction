include_directories(${CMAKE_SOURCE_DIR}/src/lib)
include_directories(${CMAKE_SOURCE_DIR}/src)

set(source_files main.cpp)

#Déclaration de l'exécutable

set(EXEC_NAME test_drawshape)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

add_executable(${EXEC_NAME} ${source_files})

target_link_libraries(${EXEC_NAME} ${MATHTOOLS_LIB}
								   ${SHAPE_LIB}
								   ${UTILS_LIB}
								   ${OpenCV_LIBS})

