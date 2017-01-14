# - Try to find Mechdyne TrackD tracking API
# Once done this module will define
#  TRACKD_FOUND
#  TRACKD_INCLUDE_DIRS
#  TRACKD_LIBRARIES
#  TRACKD_DEFINITIONS
# You can set a helper
#  TRACKD_ROOT_DIR

# platform properties
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(BITS_64 ON)
endif()

# directories to search in
set(TRACKD_ROOT_DIR $ENV{TRACKD_ROOT_DIR} CACHE PATH "root directory of Mechdyne TrackD API")
set(FIND_HINT_DIRS
	${TRACKD_ROOT_DIR}
)

# headers
find_path(TRACKD_INCLUDE_DIR
	NAMES trackdAPI_CC.h
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES include
	DOC "include path"
)

# library
# for now, it searches static libs linked against dynamic RT (/MD) under windows
if(BITS_64)
	set(PATH_SUFFIX lib64)
else()
	set(PATH_SUFFIX lib)
endif()

find_library(TRACKD_LIBRARY_RELEASE
	NAMES trackdAPI_MDs trackdAPI_CC
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES ${PATH_SUFFIX}
	DOC "library"
)

find_library(TRACKD_LIBRARY_DEBUG
	NAMES trackdAPI_MDsd trackdAPI_CC
	PATHS ${FIND_HINT_DIRS}
	PATH_SUFFIXES ${PATH_SUFFIX}
	DOC "library"
)

# assemble GLUT_LIBRARY with configurations (debug, release)
include(SelectLibraryConfigurations)
select_library_configurations(TRACKD)
# I think the set(... CACHE ...) in select_library_configurations is a bug. Reported.
# Workaround:
set(TRACKD_LIBRARY_TMP ${TRACKD_LIBRARY})
unset(TRACKD_LIBRARY CACHE)
set(TRACKD_LIBRARY ${TRACKD_LIBRARY_TMP})
unset(TRACKD_LIBRARY_TMP)
# workaround for bug in cmake <= 2.8.9:
if(NOT TRACKD_LIBRARY AND "${TRACKD_LIBRARY_RELEASE}" STREQUAL "${TRACKD_LIBRARY_DEBUG}")
	    set(TRACKD_LIBRARY "${TRACKD_LIBRARY_RELEASE}")
endif()

# mark all cached variables as advanced except TRACKD_ROOT_DIR
mark_as_advanced(
	TRACKD_INCLUDE_DIR
	TRACKD_LIBRARY_DEBUG
	TRACKD_LIBRARY_RELEASE
)

# results
set(TRACKD_DEFINITIONS "")
set(TRACKD_INCLUDE_DIRS ${TRACKD_INCLUDE_DIR})
set(TRACKD_LIBRARIES ${TRACKD_LIBRARY})

# set x_FOUND
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TRACKD
	REQUIRED_VARS TRACKD_LIBRARY TRACKD_INCLUDE_DIR
	FAIL_MESSAGE "Tip: set TRACKD_ROOT_DIR or set the advanced variables directly"
)
