# - Try to find the artoolkit5 library
# Once done this will define
#
#  ARTOOLKIT5_FOUND - system has ARTOOLKIT5
#  ARTOOLKIT5_INCLUDE_DIR - the ARTOOLKIT5 include directory
#  ARTOOLKIT5_LIBRARIES - Link these to use ARTOOLKIT5
#  ARTOOLKIT5_LINK_DIRECTORIES - link directories, useful for rpath
#

if (ARTOOLKIT5_INCLUDE_DIR AND ARTOOLKIT5_LIBRARIES)

	# in cache already
	SET(ARTOOLKIT5_FOUND TRUE)

else (ARTOOLKIT5_INCLUDE_DIR AND ARTOOLKIT5_LIBRARIES)

	if (WIN32)

		# check whether the ARTOOLKIT5DIR environment variable is set and points to a 
		# valid windows ARTOOLKIT5 installation
		FIND_PATH(
			ARTOOLKIT5_DIR include/AR/ar.h
			PATHS $ENV{ARTOOLKIT5_ROOT}/
			DOC "The main directory of the ARTOOLKIT5 library, containing the subfolders include and lib" )

		if (ARTOOLKIT5_DIR)
			SET(ARTOOLKIT5_INCLUDE_DIR ${ARTOOLKIT5_DIR}/include CACHE PATH "ARTOOLKIT5 include directory")
			SET(ARTOOLKIT5_LINK_DIRECTORIES ${ARTOOLKIT5_DIR}/lib CACHE PATH "ARTOOLKIT5 link directory")
			FIND_LIBRARY(ARTOOLKIT5_LIBRARIES
				NAME
				libARWrapper.so
				PATHS
				${ARTOOLKIT5_DIR}/lib)
			SET(ARTOOLKIT5_FOUND TRUE)
		else (ARTOOLKIT5_DIR)
			SET (ARTOOLKIT5_FOUND FALSE)
		endif (ARTOOLKIT5_DIR)

	else (WIN32)

		FIND_PATH(
			ARTOOLKIT5_DIR include/AR/ar.h
			PATHS $ENV{ARTOOLKIT5_ROOT}/
			DOC "The main directory of the ARTOOLKIT5 library, containing the subfolders include and lib" )

		if (ARTOOLKIT5_DIR)
		FIND_PATH(ARTOOLKIT5_INCLUDE_DIR
			AR/ar.h
			PATHS $ENV{ARTOOLKIT5_ROOT}/include
			/usr/include)
		# message(STATUS "Found ARTOOLKIT5: ${ARTOOLKIT5_INCLUDE_DIR}")     
			FIND_LIBRARY(ARTOOLKIT5_LIBRARIES
				NAME
				libARWrapper.so
				PATHS
				${ARTOOLKIT5_DIR}/lib)
		endif (ARTOOLKIT5_DIR)
		
	endif (WIN32)

	if(ARTOOLKIT5_LIBRARIES)
		set(ARTOOLKIT5_FOUND TRUE)
	endif(ARTOOLKIT5_LIBRARIES)
	if (ARTOOLKIT5_FOUND)
		message(STATUS "Found ARTOOLKIT5: ${ARTOOLKIT5_LIBRARIES}")
	else (ARTOOLKIT5_FOUND)
		if (ARToolKit5_FIND_REQUIRED)
			message(FATAL_ERROR "Could NOT find ARTOOLKIT5")
		else (ARToolKit5_FIND_REQUIRED)
			message(STATUS "Could NOT find ARTOOLKIT5")
		endif (ARToolKit5_FIND_REQUIRED)
	endif (ARTOOLKIT5_FOUND)


endif (ARTOOLKIT5_INCLUDE_DIR AND ARTOOLKIT5_LIBRARIES)
