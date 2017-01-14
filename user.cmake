# Rename this file to, e.g. linux_64.cmake or vs11_32.cmake, and configure your default settings below.
# In your build-tree-directory call 'cmake -C this-file-with-path path-to-source-code'.
set(USER_CMAKE_TEXT "initialized by '${CMAKE_CURRENT_LIST_FILE}'")

#################################################################################
##### 					WHAT YOU CAN EASILY CHANGE 							#####

# Should your program be compiled with 64bit? If yes, set "FALSE" to "TRUE" in 
# the next line. Be sure you have the right libraries in C:/libraries/install/...!
set(User_use64BitCompiler FALSE CACHE BOOL ${USER_CMAKE_TEXT})

# How should your program be named? Change "MyProject" in the next line
set(User_ProjectName "MyProject" CACHE STRING ${USER_CMAKE_TEXT})

# Do you want to build a "Debug" or a "Release" version? Change "Release"
# in the next line if necessary. Please avoid Typos!
set(User_Build_Configuration Release CACHE STRING ${USER_CMAKE_TEXT})


##################################################################################


#####				From here on it gets more complicated					######
#####		If you dont know what you are doing, stop it from here!!!!		######

##################################################################################


set(compiler64 "")
set(libPathArchitecture "vs11_32")
set(boost_ver "")
if(User_use64BitCompiler)
	set(compiler64 " Win64")
	set(libPathArchitecture "vs11_64" CACHE STRING ${USER_CMAKE_TEXT})
	set(boost_ver "_1_57_0")
else()
	set(libPathArchitecture "vs11_32" CACHE STRING ${USER_CMAKE_TEXT})
endif()


set(USER_CMAKE_TEXT "initialized by '${CMAKE_CURRENT_LIST_FILE}'")

set(CMAKE_CONFIGURATION_TYPES ${User_Build_Configuration} CACHE STRING ${USER_CMAKE_TEXT})
if(WIN32)	
	set(CMAKE_GENERATOR "Visual Studio 11${compiler64}" CACHE INTERNAL ${USER_CMAKE_TEXT})
endif()
set(CMAKE_INSTALL_PREFIX "." CACHE PATH ${USER_CMAKE_TEXT})

#set(Boost_USE_MULTITHREADED ON CACHE BOOL ${USER_CMAKE_TEXT})
set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL ${USER_CMAKE_TEXT})
set(Boost_USE_STATIC_LIBS ON CACHE BOOL ${USER_CMAKE_TEXT})


if(WIN32)
  set(BOOST_ROOT "C:/libraries/install/${libPathArchitecture}/boost${boost_ver}"  CACHE PATH ${USER_CMAKE_TEXT})
  set(OPENSG_ROOT "C:/libraries/install/${libPathArchitecture}/opensg" CACHE PATH ${USER_CMAKE_TEXT})
  set(inVRs_ROOT_DIR "C:/libraries/install/${libPathArchitecture}/invrs" CACHE PATH ${USER_CMAKE_TEXT})
  set(CAVESceneManager_DIR "C:/libraries/install/${libPathArchitecture}/invrs/cmake" CACHE PATH ${USER_CMAKE_TEXT})
  set(VRPN_ROOT_DIR "C:/libraries/install/${libPathArchitecture}/vrpn" CACHE PATH ${USER_CMAKE_TEXT})
  set(GLUT_ROOT_PATH "C:/libraries/install/${libPathArchitecture}/freeglut/" CACHE PATH ${USER_CMAKE_TEXT})
  if("${User_Build_Configuration}" STREQUAL "Release")
	set(glutlib "freeglut.lib")
  elseif("${User_Build_Configuration}" STREQUAL "Debug")
	set(glutlib "freeglutd.lib")
  else()
	MESSAGE(FATAL_ERROR "Please check for typos in file user.cmake at the variable \"User_Build_Configuration\" ")
  endif()
  
  set(GLUT_glut_LIBRARY "${GLUT_ROOT_PATH}/lib/${glutlib}" CACHE FILEPATH ${USER_CMAKE_TEXT})
else()
  set(BOOST_ROOT /sw-5.1.0/boost/1.53.0  CACHE PATH ${USER_CMAKE_TEXT})
  set(OPENSG_ROOT /sw-5.1.0/opensg/2.0/2015-01-16 CACHE PATH ${USER_CMAKE_TEXT})
  set(inVRs_ROOT_DIR /sw-5.1.0/invrs/1.0alpha5/rev2647 CACHE PATH ${USER_CMAKE_TEXT})
  set(CAVESceneManager_DIR /sw-5.1.0/invrs/1.0alpha5/rev2647/lib/cmake/CAVESceneManager CACHE PATH ${USER_CMAKE_TEXT})
  set(VRPN_ROOT_DIR /sw-5.1.0/vrpn/7.33 CACHE PATH ${USER_CMAKE_TEXT})
endif()



# set optionals on (default off)
set(ENABLE_VRPN_SUPPORT ON CACHE BOOL "Support for VRPN Tracking")
#set(ENABLE_TRACKD_SUPPORT ON CACHE BOOL "Support for TrackD Tracking")
set(ENABLE_INVRSVRPNDEVICE_SUPPORT ON CACHE BOOL "Support for inVRsVrpnDevice Tracking")
#set(ENABLE_INVRSTRACKDDEVICE_SUPPORT ON CACHE BOOL "Support for inVRsTrackdDevice Tracking")

