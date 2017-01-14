# - Try to find VRPN
# Once done this module will define
#  VRPN_FOUND
#  VRPN_INCLUDE_DIRS
#  VRPN_LIBRARIES
#  VRPN_DEFINITIONS
# You can set a helper
#  VRPN_ROOT_DIR

# platform properties
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(BITS_64 ON)
endif()

# directories to search in
set(VRPN_ROOT_DIR $ENV{VRPN_ROOT_DIR} CACHE PATH "root directory of the API")
set(FIND_HINT_DIRS
	${VRPN_ROOT_DIR}
)

# headers
find_path(VRPN_INCLUDE_DIR
	NAMES vrpn_Tracker.h
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES include
	DOC "include path"
)

# library
if(NOT WIN32)
	if(BITS_64)
		set(PATH_SUFFIX lib64)
	else()
		set(PATH_SUFFIX lib)
	endif()
else()
	set(PATH_SUFFIX lib)
endif()
find_library(VRPN_LIBRARY_RELEASE
	NAMES vrpn
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES ${PATH_SUFFIX}
	DOC "library"
)

find_library(VRPN_LIBRARY_DEBUG
	NAMES vrpnd
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES ${PATH_SUFFIX}
	DOC "library"
)

# # gpsnmealib
# # headers
# find_path(GPSNMEA_INCLUDE_DIR
	# NAMES gpsnmealib/nmeaParser.h
	# PATHS ${FIND_HINT_DIRS}
	# DOC "include path"
# )
# mark_as_advanced(GPSNMEA_INCLUDE_DIR)
# set(VRPN_INCLUDE_DIRS ${GPSNMEA_INCLUDE_DIR})

# # library
# find_library(GPSNMEA_LIBRARY_RELEASE
	# NAMES gpsnmealib
	# PATHS ${FIND_HINT_DIRS}/Release
	# DOC "library"
# )

# find_library(GPSNMEA_LIBRARY_DEBUG
	# NAMES gpsnmealib
	# PATHS ${FIND_HINT_DIRS}/Debug
	# DOC "library"
# )

# assemble x_LIBRARY and x_LIBRARIES with configurations (debug, release)
include(SelectLibraryConfigurations)
select_library_configurations(VRPN)
select_library_configurations(GPSNMEA)
# Workaround: Remove x_LIBRARY from the cache
set(VRPN_LIBRARY_TMP ${VRPN_LIBRARY})
unset(VRPN_LIBRARY CACHE)
set(VRPN_LIBRARY ${VRPN_LIBRARY_TMP})
unset(VRPN_LIBRARY_TMP)
set(GPSNMEA_LIBRARY_TMP ${GPSNMEA_LIBRARY})
unset(GPSNMEA_LIBRARY CACHE)
set(GPSNMEA_LIBRARY ${GPSNMEA_LIBRARY_TMP})
unset(GPSNMEA_LIBRARY_TMP)

list(APPEND VRPN_LIBRARIES ${GPSNMEA_LIBRARIES})

# mark all cached variables as advanced except helper
mark_as_advanced(
	VRPN_INCLUDE_DIR
	VRPN_LIBRARY_DEBUG
	VRPN_LIBRARY_RELEASE
)

# results
set(VRPN_DEFINITIONS "")
set(VRPN_INCLUDE_DIRS ${VRPN_INCLUDE_DIR})
set(VRPN_LIBRARIES ${VRPN_LIBRARY})

# set x_FOUND
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(VRPN
	REQUIRED_VARS VRPN_LIBRARY VRPN_INCLUDE_DIR
	FAIL_MESSAGE "Tip: set VRPN_ROOT_DIR or set the advanced variables directly"
)
# GPSNMEA_LIBRARY GPSNMEA_INCLUDE_DIR

