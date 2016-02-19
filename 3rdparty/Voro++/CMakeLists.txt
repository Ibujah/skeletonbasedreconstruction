set(LIBRARY_NAME ${VOROPLUSPLUS_LIBRARY})

include_directories(voro++/)
					
set(SOURCE_FILES    voro++/cell.cc
					voro++/c_loops.cc
					voro++/cmd_line.cc
					voro++/common.cc
					voro++/container.cc
					voro++/container_prd.cc
					voro++/pre_container.cc
					voro++/unitcell.cc
					voro++/v_base.cc
					voro++/v_compute.cc
					voro++/wall.cc)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
