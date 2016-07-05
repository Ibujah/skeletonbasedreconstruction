add_subdirectory(userinput/)
add_subdirectory(displayopencv/)
add_subdirectory(display3d/)
add_subdirectory(fileio/)

if (DO_TRACKER)
	add_subdirectory(tracking/)
endif (DO_TRACKER)
