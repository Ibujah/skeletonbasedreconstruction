include_directories(${CMAKE_SOURCE_DIR}/src/lib
					${Boost_INCLUDE_DIR})

set(source_files main.cpp)

#Déclaration de l'exécutable

set(EXEC_NAME test_skeleton)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

add_executable(${EXEC_NAME} ${source_files})

target_link_libraries(${EXEC_NAME} ${MATHTOOLS_LIB}
								   ${SKELETON_LIB}
								   ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
