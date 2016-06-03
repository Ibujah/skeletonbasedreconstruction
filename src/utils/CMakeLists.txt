add_subdirectory(userinput/)
add_subdirectory(displayopencv/)
add_subdirectory(display3d/)
add_subdirectory(fileio/)

if (ARTOOLKIT5_FOUND)
	add_subdirectory(tracking/)
endif (ARTOOLKIT5_FOUND)
