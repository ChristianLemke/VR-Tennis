cmake_minimum_required(VERSION 2.8)


project(${User_ProjectName})



list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
include(AddProject)

set(OPENSG_MODULES OSGBase OSGSystem OSGWindow OSGWindowGLUT OSGDrawable OSGUtil OSGGroup OSGFileIO OSGImageFileIO)

set(EXECUTABLE_OUTPUT_PATH ../exe/)
add_project(${PROJECT_NAME}
	SOURCE_DIR "sources/"
	HEADER_DIR "includes/"
	CONFIGS "config/mono.csm"
	OPENSG_MODULES ${OPENSG_MODULES}
	CSM_MODULE
	VRPN_MODULE
)


if(WIN32)
	if("${CMAKE_CONFIGURATION_TYPES}" STREQUAL "Release")
		set(confTypeFolder "rel")
	elseif("${CMAKE_CONFIGURATION_TYPES}" STREQUAL "Debug")
		set(confTypeFolder "debug")
	else()
		 message( FATAL_ERROR "The build type ${CMAKE_CONFIGURATION_TYPES} is not supported")
	endif()
	set(OPENSG_BIN "C:\\libraries\\install\\${libPathArchitecture}\\opensg\\bin\\${confTypeFolder}")
	set(GLUT_BIN "C:\\libraries\\install\\${libPathArchitecture}\\freeglut\\bin")
	set(INVRS_BIN "C:\\libraries\\install\\${libPathArchitecture}\\invrs\\bin")
	set(TIFF_BIN "C:\\libraries\\install\\${libPathArchitecture}\\libtiff\\bin")
	set(PNG_BIN "C:\\libraries\\install\\${libPathArchitecture}\\libpng\\bin")
	set(ZLIB_BIN "C:\\libraries\\install\\${libPathArchitecture}\\zlib\\bin")
	set(JPEG_BIN "C:\\libraries\\install\\${libPathArchitecture}\\libjpeg-turbo\\bin")
	
	set(TEXT_OPENSG_CSM "SET opensg_BIN=C:\\libraries\\install\\${libPathArchitecture}\\opensg\\bin\\rel\n") # as csm is only compiled in release!
	set(TEXT_OPENSG_OTHER "SET opensg_BIN=${OPENSG_BIN}\n")
	
	set(TEXT_FOR_PATH "SET glut_bin=${GLUT_BIN}\nSET invrs_bin=${INVRS_BIN}\n")
	set(TEXT_FOR_PATH "${TEXT_FOR_PATH}SET tiff_bin=${TIFF_BIN}\nSET png_bin=${PNG_BIN}\nSET zlib_bin=${ZLIB_BIN}\n")
	set(TEXT_FOR_PATH "${TEXT_FOR_PATH}SET jpeg_bin=${JPEG_BIN}\n")	
	set(TEXT_FOR_PATH "${TEXT_FOR_PATH}SET PATH=%opensg_BIN%;%glut_bin%;%invrs_bin%;%tiff_bin%;%png_bin%;%zlib_bin%;%jpeg_bin%;%PATH%;")
	
	
	file(WRITE start_demo_${CMAKE_CONFIGURATION_TYPES}.bat "${TEXT_OPENSG_OTHER}${TEXT_FOR_PATH} \n.\\exe\\${CMAKE_CONFIGURATION_TYPES}\\${PROJECT_NAME}.exe -f config\\mono.csm" )
	file(WRITE start_demo_cave_${CMAKE_CONFIGURATION_TYPES}.bat "${TEXT_OPENSG_OTHER}${TEXT_FOR_PATH} \n.\\exe\\${CMAKE_CONFIGURATION_TYPES}\\${PROJECT_NAME}.exe -f config\\lrz_cave_vrpn.csm" )
	file(WRITE start_csm.bat "${TEXT_OPENSG_CSM}${TEXT_FOR_PATH} \nstart \"front cmd\" \"csmrenderserver.exe\" -a 127.0.0.1 -g 800x600+100+100 -w front" )
else()
	file(WRITE start_demo_${CMAKE_CONFIGURATION_TYPES} "./exe/${PROJECT_NAME} -f /sw/config/opensg/csm/mono.csm" )
	file(WRITE start_demo_cave_${CMAKE_CONFIGURATION_TYPES} "./exe/${PROJECT_NAME} -f /sw/config/opensg/csm/lrz_cave_vrpn.csm" )
	execute_process(COMMAND "chmod +x start_demo_${CMAKE_CONFIGURATION_TYPES}")
	execute_process(COMMAND "chmod +x start_demo_cave_${CMAKE_CONFIGURATION_TYPES}")
endif()
