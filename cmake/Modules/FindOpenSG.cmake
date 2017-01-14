# - Find OpenSG 2 libraries
# Find the specified OpenSG 2 libraries and header files.
#
# Since OpenSG consists of a number of libraries you need to specify which
# of those you want to use. To do so, pass a list of their names after
# the COMPONENTS argument to FIND_PACKAGE. A typical call looks like this:
# FIND_PACKAGE(OpenSG REQUIRED COMPONENTS OSGBase OSGSystem OSGDrawable)
#
# This module specifies the following variables:
#  OpenSG_INCLUDE_DIRS
#  OpenSG_LIBRARIES
#  OpenSG_LIBRARY_DIRS
#  OpenSG_VERSION
#  OpenSG_MAJOR_VERSION
#  OpenSG_MINOR_VERSION
#  OpenSG_RELEASE_VERSION
#
#  For each component COMP the capitalized name (e.g. OSGBASE, OSGSYSTEM):
#  OpenSG_${COMP}_LIBRARY
#  OpenSG_${COMP}_LIBRARY_RELEASE
#  OpenSG_${COMP}_LIBRARY_DEBUG
#
#  You can control where this module attempts to locate libraries and headers:
#  you can use the following input variables:
#  OPENSG_ROOT          root of an installed OpenSG with include/OpenSG and lib below it
#  OPENSG_INCLUDE_DIR   header directory
#  OPENSG_LIBRARY_DIR   library directory

# OpenSG_DIR is what is automaticall added by cmake

# use OPENSG_ROOT from environment, if available:
IF ( NOT OPENSG_ROOT )
	IF ( IS_DIRECTORY "$ENV{OPENSG_ROOT}" )
		set(OPENSG_ROOT "$ENV{OPENSG_ROOT}")
		#message(STATUS "Using OPENSG_ROOT from environment: ${OPENSG_ROOT}")
	ENDIF ( IS_DIRECTORY "$ENV{OPENSG_ROOT}" )
ENDIF ( NOT OPENSG_ROOT )

#dependency map from osg2-config but full expanded
MACRO(OSG2_DEF_COMPONENT name)
	set(OpenSG_OSG${name}_deps OSG${name})
	FOREACH(dep ${ARGN})
		set(OpenSG_OSG${name}_deps ${OpenSG_OSG${name}_deps} ${OpenSG_OSG${dep}_deps} )
	ENDFOREACH(dep)
	list(REMOVE_DUPLICATES OpenSG_OSG${name}_deps)
ENDMACRO(OSG2_DEF_COMPONENT)

#alias map from osg2 to osg1
MACRO(OSG2_TO_1_MAPPING from to)
	set(OpenSG_OSG${from}_2to1 OSG${to})
ENDMACRO(OSG2_TO_1_MAPPING)


OSG2_DEF_COMPONENT(Base)
OSG2_DEF_COMPONENT(System Base)
OSG2_DEF_COMPONENT(ImageFileIO System)
OSG2_DEF_COMPONENT(State System)
OSG2_DEF_COMPONENT(ContribComputeBase System)
OSG2_DEF_COMPONENT(WindowWIN32 Base System)
OSG2_DEF_COMPONENT(WindowX Base System)
OSG2_DEF_COMPONENT(WindowQt Base System)
if(WIN32)
	OSG2_DEF_COMPONENT(WindowGLUT WindowWIN32)
	OSG2_DEF_COMPONENT(Window System WindowWIN32)
else()
	OSG2_DEF_COMPONENT(WindowGLUT WindowX)
	OSG2_DEF_COMPONENT(Window System WindowX)
endif()
OSG2_DEF_COMPONENT(Group State Window)
OSG2_DEF_COMPONENT(ContribTrapezoidalShadowMaps Group)
OSG2_DEF_COMPONENT(Drawable State Group)
OSG2_DEF_COMPONENT(Text Drawable)
OSG2_DEF_COMPONENT(EffectGroups Drawable Group State Window)
OSG2_DEF_COMPONENT(Cluster Drawable Group State Window)
OSG2_DEF_COMPONENT(Dynamics Drawable Group)
OSG2_DEF_COMPONENT(FileIO Drawable Group State Dynamics)
OSG2_DEF_COMPONENT(ContribGUI Drawable Group State)
OSG2_DEF_COMPONENT(ContribPLY Drawable)
OSG2_DEF_COMPONENT(Util Drawable Group State Text Window)
OSG2_DEF_COMPONENT(ContribBackgroundLoader Drawable Util)
OSG2_DEF_COMPONENT(ContribCSM Drawable Window FileIO Dynamics Util Cluster WindowGLUT)
OSG2_DEF_COMPONENT(ContribCSMSimplePlugin ContribCSM)

#OpenSG2 to OpenSG1 component mapping
OSG2_TO_1_MAPPING(ImageFileIO System)
OSG2_TO_1_MAPPING(State System)
OSG2_TO_1_MAPPING(ContribComputeBase System)
if(WIN32)
	OSG2_TO_1_MAPPING(Window WindowWIN32)
else()
	OSG2_TO_1_MAPPING(Window WindowX)
endif()
OSG2_TO_1_MAPPING(Group System)
OSG2_TO_1_MAPPING(ContribTrapezoidalShadowMaps System)
OSG2_TO_1_MAPPING(Drawable System)
OSG2_TO_1_MAPPING(Text System)
OSG2_TO_1_MAPPING(EffectGroups System)
OSG2_TO_1_MAPPING(Cluster System)
OSG2_TO_1_MAPPING(Dynamics System)
OSG2_TO_1_MAPPING(FileIO System)
OSG2_TO_1_MAPPING(ContribGUI System)
OSG2_TO_1_MAPPING(ContribPLY System)
OSG2_TO_1_MAPPING(Util System)
OSG2_TO_1_MAPPING(ContribBackgroundLoader System)
OSG2_TO_1_MAPPING(ContribCSM System)
OSG2_TO_1_MAPPING(ContribCSMSimplePlugin System)

IF(NOT OPENSG_ROOT AND OpenSG_DIR)
	SET(OPENSG_ROOT ${OpenSG_DIR})
ENDIF(NOT OPENSG_ROOT AND OpenSG_DIR)

#resolve indirect dependencies between components
FOREACH(COMPONENT ${OpenSG_FIND_COMPONENTS})
	SET(OpenSG_FIND_RESOLVED_COMPONENTS
		${OpenSG_FIND_RESOLVED_COMPONENTS}
		${COMPONENT}
		${OpenSG_${COMPONENT}_deps})
ENDFOREACH(COMPONENT)
if(OpenSG_FIND_RESOLVED_COMPONENTS)
	list(REMOVE_DUPLICATES OpenSG_FIND_RESOLVED_COMPONENTS)
endif(OpenSG_FIND_RESOLVED_COMPONENTS)

IF(NOT OSG_FORCE_CACHE_UPDATE)
	SET(__OpenSG_IN_CACHE TRUE)
	IF(OpenSG_INCLUDE_DIR)
		FOREACH(COMPONENT ${OpenSG_FIND_RESOLVED_COMPONENTS})
			STRING(TOUPPER ${COMPONENT} COMPONENT)
			IF(NOT OpenSG_${COMPONENT}_FOUND)
				SET(__OpenSG_IN_CACHE FALSE)
			ENDIF(NOT OpenSG_${COMPONENT}_FOUND)
		ENDFOREACH(COMPONENT)
	ELSE(OpenSG_INCLUDE_DIR)
		SET(__OpenSG_IN_CACHE FALSE)
	ENDIF(OpenSG_INCLUDE_DIR)
ENDIF(NOT OSG_FORCE_CACHE_UPDATE)

# The reason that we failed to find OpenSG. This will be set to a
# user-friendly message when we fail to find some necessary piece of
# OpenSG.
set(OpenSG_ERROR_REASON)

############################################
#
# Check the existence of the libraries.
#
############################################
# This macro is directly taken from FindBoost.cmake that comes with the cmake
# distribution. It is NOT my work, only minor modifications have been made to
# remove references to boost.
#########################################################################

MACRO(__OpenSG_ADJUST_LIB_VARS basename)
	IF(OpenSG_INCLUDE_DIR)
		IF(OpenSG_${basename}_LIBRARY_DEBUG AND OpenSG_${basename}_LIBRARY_RELEASE)
			# if the generator supports configuration types then set
			# optimized and debug libraries, or if the CMAKE_BUILD_TYPE has a value
			IF (CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
				SET(OpenSG_${basename}_LIBRARY debug ${OpenSG_${basename}_LIBRARY_DEBUG} optimized ${OpenSG_${basename}_LIBRARY_RELEASE})
			ELSE(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
				# if there are no configuration types and CMAKE_BUILD_TYPE has no value
				# then just use the release libraries
				SET(OpenSG_${basename}_LIBRARY ${OpenSG_${basename}_LIBRARY_RELEASE} )
			ENDIF(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)

			SET(OpenSG_${basename}_LIBRARIES debug ${OpenSG_${basename}_LIBRARY_DEBUG} optimized ${OpenSG_${basename}_LIBRARY_RELEASE})
			GET_FILENAME_COMPONENT(OpenSG_${basename}_LIBRARY_DIRS "${OpenSG_${basename}_LIBRARY_RELEASE}" PATH)
		ENDIF(OpenSG_${basename}_LIBRARY_DEBUG AND OpenSG_${basename}_LIBRARY_RELEASE)

		# if only the release version was found, set the debug variable also to the release version
		IF(OpenSG_${basename}_LIBRARY_RELEASE AND NOT OpenSG_${basename}_LIBRARY_DEBUG)
			SET(OpenSG_${basename}_LIBRARY       ${OpenSG_${basename}_LIBRARY_RELEASE})
			SET(OpenSG_${basename}_LIBRARIES     ${OpenSG_${basename}_LIBRARY_RELEASE})
			GET_FILENAME_COMPONENT(OpenSG_${basename}_LIBRARY_DIRS "${OpenSG_${basename}_LIBRARY}" PATH)
		ENDIF(OpenSG_${basename}_LIBRARY_RELEASE AND NOT OpenSG_${basename}_LIBRARY_DEBUG)

		# if only the debug version was found, set the release variable also to the debug version
		IF(OpenSG_${basename}_LIBRARY_DEBUG AND NOT OpenSG_${basename}_LIBRARY_RELEASE)
			SET(OpenSG_${basename}_LIBRARY         ${OpenSG_${basename}_LIBRARY_DEBUG})
			SET(OpenSG_${basename}_LIBRARIES       ${OpenSG_${basename}_LIBRARY_DEBUG})
			GET_FILENAME_COMPONENT(OpenSG_${basename}_LIBRARY_DIRS "${OpenSG_${basename}_LIBRARY}" PATH)
		ENDIF(OpenSG_${basename}_LIBRARY_DEBUG AND NOT OpenSG_${basename}_LIBRARY_RELEASE)

		IF(OpenSG_${basename}_LIBRARY)
			SET(OpenSG_${basename}_LIBRARY ${OpenSG_${basename}_LIBRARY} CACHE FILEPATH "The OpenSG ${basename} library")
			SET(OpenSG_${basename}_LIBRARIES ${OpenSG_${basename}_LIBRARIES} CACHE FILEPATH "The OpenSG ${basename} libraries")
			SET(OpenSG_LIBRARY_DIRS ${OpenSG_LIBRARY_DIRS} ${OpenSG_${basename}_LIBRARY_DIRS} CACHE FILEPATH "OpenSG library directory")
			SET(OpenSG_${basename}_FOUND ON CACHE INTERNAL "Whether the OpenSG ${basename} library found")
		ENDIF(OpenSG_${basename}_LIBRARY)
	ENDIF(OpenSG_INCLUDE_DIR)

	# Make variables changeble to the advanced user
	MARK_AS_ADVANCED(
		OpenSG_${basename}_LIBRARY
		OpenSG_${basename}_LIBRARIES
		OpenSG_${basename}_FOUND
		OpenSG_${basename}_LIBRARY_RELEASE
		OpenSG_${basename}_LIBRARY_DEBUG
	)
ENDMACRO(__OpenSG_ADJUST_LIB_VARS)


FUNCTION(SETUP_OSG_LIB_TARGETS COMPONENT UPPERCOMPONENT)

	IF ( TARGET ${COMPONENT} )
		# only setup target, if target is not defined yet:
		# (i.e. allow this script to be called twice without errors)
		RETURN()
	ENDIF ( TARGET ${COMPONENT} )

	ADD_LIBRARY(${COMPONENT} SHARED IMPORTED)

	IF(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE)

		GET_FILENAME_COMPONENT(OSG_TMP_LIB_DIR_OPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE}
							PATH)

		IF(UNIX)
			SET(OSG_IMP_RELEASE IMPORTED_LOCATION_RELEASE ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE})
		ELSE(UNIX)
			SET(OSG_IMP_RELEASE IMPORTED_IMPLIB_RELEASE ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE}
							IMPORTED_LOCATION_RELEASE ${OSG_TMP_LIB_DIR_OPT})
		ENDIF(UNIX)

	ENDIF(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE)


	IF(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT)

		GET_FILENAME_COMPONENT(OSG_TMP_LIB_DIR_OPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT}
							PATH)

		IF(UNIX)
			SET(OSG_IMP_RELEASENOOPT IMPORTED_LOCATION_RELEASENOOPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT})
		ELSE(UNIX)
			SET(OSG_IMP_RELEASENOOPT IMPORTED_IMPLIB_RELEASENOOPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT}
								IMPORTED_LOCATION_RELEASENOOPT ${OSG_TMP_LIB_DIR_OPT})
		ENDIF(UNIX)

	ENDIF(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT)


	IF(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG)

		GET_FILENAME_COMPONENT(OSG_TMP_LIB_DIR_DBG ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG}
							PATH)

		IF(UNIX)
			SET(OSG_IMP_DEBUG IMPORTED_LOCATION_DEBUG ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG})
			SET(OSG_IMP_DEBUG ${OSG_IMP_DEBUG} IMPORTED_LOCATION_DEBUGGV ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG})
		ELSE(UNIX)
			SET(OSG_IMP_DEBUG IMPORTED_IMPLIB_DEBUG ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG}
							IMPORTED_LOCATION_DEBUG ${OSG_TMP_LIB_DIR_DBG})
		ENDIF(UNIX)

	ENDIF(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG)


	IF(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT)

		GET_FILENAME_COMPONENT(OSG_TMP_LIB_DIR_DBG ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT}
							PATH)

		IF(UNIX)
			SET(OSG_IMP_DEBUGOPT IMPORTED_LOCATION_DEBUGOPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT})
			SET(OSG_IMP_DEBUGOPT ${OSG_IMP_DEBUGOPT} IMPORTED_LOCATION_DEBUGOPTGV ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT})
		ELSE(UNIX)
			SET(OSG_IMP_DEBUGOPT IMPORTED_IMPLIB_DEBUGOPT ${OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT}
							IMPORTED_LOCATION_DEBUGOPT ${OSG_TMP_LIB_DIR_DBG})
		ENDIF(UNIX)

	ENDIF(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT)

	IF(OSG_IMP_RELEASE OR OSG_IMP_RELEASENOOPT OR OSG_IMP_DEBUG OR OSG_IMP_DEBUGOPT)
		SET_TARGET_PROPERTIES(${COMPONENT} PROPERTIES
							${OSG_IMP_RELEASE}
							${OSG_IMP_RELEASENOOPT}
							${OSG_IMP_DEBUG}
							${OSG_IMP_DEBUGOPT}       )
	ENDIF(OSG_IMP_RELEASE OR OSG_IMP_RELEASENOOPT OR OSG_IMP_DEBUG OR OSG_IMP_DEBUGOPT)
ENDFUNCTION(SETUP_OSG_LIB_TARGETS)

FUNCTION(MAP_COMPONENT component mappedName)
	IF((OpenSG_MAJOR_VERSION STREQUAL "2") OR NOT OpenSG_${component}_2to1) #no mapping in OpenSG2
		SET(${mappedName} ${component} PARENT_SCOPE)
	ELSE()
		SET(${mappedName} ${OpenSG_${component}_2to1} PARENT_SCOPE)
	ENDIF()
ENDFUNCTION(MAP_COMPONENT)

#-------------------------------------------------------------------------------
IF(__OpenSG_IN_CACHE)
	# values are already in the cache

	SET(OpenSG_FOUND TRUE)
	FOREACH(COMPONENT ${OpenSG_FIND_RESOLVED_COMPONENTS})
		STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)

		__OpenSG_ADJUST_LIB_VARS(${UPPERCOMPONENT})
		SET(OpenSG_LIBRARIES ${OpenSG_LIBRARIES} ${OpenSG_${UPPERCOMPONENT}_LIBRARY})
		SETUP_OSG_LIB_TARGETS(${COMPONENT} ${UPPERCOMPONENT})
	ENDFOREACH(COMPONENT)

	SET(OpenSG_INCLUDE_DIRS "${OpenSG_INCLUDE_DIR}" "${OpenSG_INCLUDE_DIR}/OpenSG")

	# If OpenSG2 -> find Boost header and add to include DIRS
	IF(OpenSG_MAJOR_VERSION STREQUAL "2")
		# Find BOOST headers needed for OpenSG2
		find_package(Boost COMPONENTS filesystem system REQUIRED QUIET)
		SET(OpenSG_INCLUDE_DIRS ${OpenSG_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
		SET(OpenSG_LIBRARY_DIRS ${OpenSG_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
		set(OpenSG_LIBRARIES ${OpenSG_LIBRARIES} ${Boost_LIBRARIES})
		set(OpenSG_DEFINITIONS ${OpenSG_DEFINITIONS} ${Boost_DEFINITIONS})
	ENDIF(OpenSG_MAJOR_VERSION STREQUAL "2")
ELSE(__OpenSG_IN_CACHE)
	# need to search for libs

	SET(__OpenSG_INCLUDE_SEARCH_DIRS
		C:/OpenSG/include
		"C:/OpenSG"
		"$ENV{ProgramFiles}/OpenSG"
		/sw/local/include
	)

	SET(__OpenSG_LIBRARIES_SEARCH_DIRS
		C:/OpenSG/lib
		"C:/OpenSG"
		"$ENV{ProgramFiles}/OpenSG"
		/sw/local/lib
	)

	# handle input variable OPENSG_ROOT
	IF(OPENSG_ROOT)
		FILE(TO_CMAKE_PATH ${OPENSG_ROOT} OPENSG_ROOT)
		SET(__OpenSG_INCLUDE_SEARCH_DIRS
			${OPENSG_ROOT}/include
			${OPENSG_ROOT}
			${__OpenSG_INCLUDE_SEARCH_DIRS})

		SET(__OpenSG_LIBRARIES_SEARCH_DIRS
			${OPENSG_ROOT}/lib
			${OPENSG_ROOT}/lib64
			${OPENSG_ROOT}/stage/lib
			${__OpenSG_LIBRARIES_SEARCH_DIRS})
	ENDIF(OPENSG_ROOT)

	# handle input variable OPENSG_INCLUDE_DIR
	IF(OPENSG_INCLUDE_DIR)
		FILE(TO_CMAKE_PATH ${OPENSG_INCLUDE_DIR} OPENSG_INCLUDE_DIR)
		SET(__OpenSG_INCLUDE_SEARCH_DIRS
			${OPENSG_INCLUDE_DIR} ${__OpenSG_INCLUDE_SEARCH_DIRS})
	ENDIF(OPENSG_INCLUDE_DIR)

	# handle input variable OPENSG_LIBRARY_DIR
	IF(OPENSG_LIBRARY_DIR)
		FILE(TO_CMAKE_PATH ${OPENSG_LIBRARY_DIR} OPENSG_LIBRARY_DIR)
		SET(__OpenSG_LIBRARIES_SEARCH_DIRS
			${OPENSG_LIBRARY_DIR} ${__OpenSG_LIBRARIES_SEARCH_DIRS})
	ENDIF(OPENSG_LIBRARY_DIR)

	IF(NOT OpenSG_INCLUDE_DIR)
		# try to find include dirrectory by searching for OSGConfigured.h

		FIND_PATH(OpenSG_INCLUDE_DIR
			NAMES         OpenSG/OSGConfigured.h
			HINTS         ${__OpenSG_INCLUDE_SEARCH_DIRS})
	ENDIF(NOT OpenSG_INCLUDE_DIR)

	# ------------------------------------------------------------------------
	#  Resolving OpenSG Version
	# ------------------------------------------------------------------------
	IF(OpenSG_INCLUDE_DIR)
		# Extract OpenSG_VERSION OSGConfig.h
		FILE(READ "${OpenSG_INCLUDE_DIR}/OpenSG/OSGConfig.h" __OpenSG_VERSION_H_CONTENTS)
		STRING(REGEX REPLACE ".*#define OSG_MAJOR_VERSION +([0-9]+).*" "\\1" OpenSG_MAJOR_VERSION "${__OpenSG_VERSION_H_CONTENTS}")
		STRING(REGEX REPLACE ".*#define OSG_MINOR_VERSION +([0-9]+).*" "\\1" OpenSG_MINOR_VERSION "${__OpenSG_VERSION_H_CONTENTS}")
		STRING(REGEX REPLACE ".*#define OSG_RELEASE_VERSION +([0-9]+).*" "\\1" OpenSG_RELEASE_VERSION "${__OpenSG_VERSION_H_CONTENTS}")

		SET(OpenSG_VERSION "${OpenSG_MAJOR_VERSION}.${OpenSG_MINOR_VERSION}.${OpenSG_RELEASE_VERSION}")
		SET(OpenSG_VERSION ${OpenSG_VERSION} CACHE STRING "The version number for OpenSG libraries")
		SET(OpenSG_MAJOR_VERSION ${OpenSG_MAJOR_VERSION} CACHE STRING "The version number for OpenSG libraries")
		SET(OpenSG_MINOR_VERSION ${OpenSG_MINOR_VERSION} CACHE STRING "The version number for OpenSG libraries")
		SET(OpenSG_RELEASE_VERSION ${OpenSG_RELEASE_VERSION} CACHE STRING "The version number for OpenSG libraries")


		# Extract whether the MICROSOFT... was set
		IF(MSVC)
			FILE(READ "${OpenSG_INCLUDE_DIR}/OpenSG/OSGConfigured.h" __OpenSG_CONFIGURED_H_CONTENTS)
			STRING(REGEX MATCH ".*#define OSG_DISABLE_MICROSOFT_SECURE_CXXX.*" OSG_DISABLE_MICROSOFT_SECURE_CXXX "${__OpenSG_CONFIGURED_H_CONTENTS}")

			IF(OSG_DISABLE_MICROSOFT_SECURE_CXXX)
				set(OpenSG_DISABLE_MICROSOFT_SECURE_CXXX_DEFINITIONS
				"-D_CRT_SECURE_NO_DEPRECATE" "-D_CRT_SECURE_NO_WARNINGS"
				"-D_CRT_NONSTDC_NO_DEPRECATE" "-D_SECURE_SCL=0"
				"-D_SCL_SECURE_NO_WARNINGS" "-D_SCL_SECURE_NO_DEPRECATE" "-D_HAS_ITERATOR_DEBUGGING=0"
				CACHE STRING "The definitions from OpenSG to disable MS secure CXXX")
			ELSE()
			ENDIF()
		ENDIF(MSVC)

		mark_as_advanced(OpenSG_VERSION OpenSG_MAJOR_VERSION  OpenSG_MINOR_VERSION OpenSG_RELEASE_VERSION OpenSG_DISABLE_MICROSOFT_SECURE_CXXX_DEFINITIONS)

	ENDIF(OpenSG_INCLUDE_DIR)

	# ------------------------------------------------------------------------
	#  Begin finding OpenSG libraries
	# ------------------------------------------------------------------------
	FOREACH(COMPONENT ${OpenSG_FIND_RESOLVED_COMPONENTS})
		STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
		MAP_COMPONENT(${COMPONENT} MAPPEDCOMPONENT)
		SET(OpenSG_${UPPERCOMPONENT}_LIBRARY "OpenSG_${UPPERCOMPONENT}_LIBRARY-NOTFOUND" )
		SET(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE "OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE-NOTFOUND" )
		SET(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG "OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG-NOTFOUND")

		IF(WIN32)
			SET(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT "OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT-NOTFOUND" )
			SET(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT "OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT-NOTFOUND")
		ENDIF(WIN32)

		FIND_LIBRARY(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASE
			NAMES  ${MAPPEDCOMPONENT}
			HINTS  ${__OpenSG_LIBRARIES_SEARCH_DIRS}
			PATH_SUFFIXES "release" "rel" "opt" "relwithdbg"
		)

		IF(WIN32)
			FIND_LIBRARY(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG
				NAMES  "${MAPPEDCOMPONENT}D"
				HINTS  ${__OpenSG_LIBRARIES_SEARCH_DIRS}
				PATH_SUFFIXES "debug" "dbg" "debugopt"
			)

			FIND_LIBRARY(OpenSG_${UPPERCOMPONENT}_LIBRARY_RELEASENOOPT
				NAMES  "${MAPPEDCOMPONENT}RN"
				HINTS  ${__OpenSG_LIBRARIES_SEARCH_DIRS}
				PATH_SUFFIXES "relnoopt"
			)

			FIND_LIBRARY(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUGOPT
				NAMES  "${MAPPEDCOMPONENT}DO"
				HINTS  ${__OpenSG_LIBRARIES_SEARCH_DIRS}
				PATH_SUFFIXES "debugopt"
			)
		ELSE(WIN32)
			FIND_LIBRARY(OpenSG_${UPPERCOMPONENT}_LIBRARY_DEBUG
				NAMES  ${MAPPEDCOMPONENT}
				HINTS  ${__OpenSG_LIBRARIES_SEARCH_DIRS}
				PATH_SUFFIXES "debug" "dbg"
			)
		ENDIF(WIN32)

		__OpenSG_ADJUST_LIB_VARS(${UPPERCOMPONENT})

	ENDFOREACH(COMPONENT)
	# ------------------------------------------------------------------------
	#  End finding OpenSG libraries
	# ------------------------------------------------------------------------

	SET(OpenSG_INCLUDE_DIRS "${OpenSG_INCLUDE_DIR}" "${OpenSG_INCLUDE_DIR}/OpenSG")

	SET(OpenSG_FOUND FALSE)

	IF(OpenSG_INCLUDE_DIR)
		SET(OpenSG_FOUND TRUE)

		# check if all requested components were found
		SET(__OpenSG_CHECKED_COMPONENT FALSE)
		SET(__OpenSG_MISSING_COMPONENTS)

		FOREACH(COMPONENT ${OpenSG_FIND_RESOLVED_COMPONENTS})
			STRING(TOUPPER ${COMPONENT} COMPONENT)
			SET(__OpenSG_CHECKED_COMPONENT TRUE)

			IF(NOT OpenSG_${COMPONENT}_FOUND)
				STRING(TOLOWER ${COMPONENT} COMPONENT)
				LIST(APPEND __OpenSG_MISSING_COMPONENTS ${COMPONENT})
				SET(OpenSG_FOUND FALSE)
			ENDIF(NOT OpenSG_${COMPONENT}_FOUND)
		ENDFOREACH(COMPONENT)

		IF(__OpenSG_MISSING_COMPONENTS)
			# We were unable to find some libraries, so generate a sensible
			# error message that lists the libraries we were unable to find.
			SET(OpenSG_ERROR_REASON
				"${OpenSG_ERROR_REASON}\nThe following OpenSG libraries could not be found:\n")
			FOREACH(COMPONENT ${__OpenSG_MISSING_COMPONENTS})
				SET(OpenSG_ERROR_REASON
					"${OpenSG_ERROR_REASON}        ${COMPONENT}\n")
			ENDFOREACH(COMPONENT)

			LIST(LENGTH OpenSG_FIND_COMPONENTS __OpenSG_NUM_COMPONENTS_WANTED)
			LIST(LENGTH __OpenSG_MISSING_COMPONENTS __OpenSG_NUM_MISSING_COMPONENTS)
			IF(${__OpenSG_NUM_COMPONENTS_WANTED} EQUAL ${__OpenSG_NUM_MISSING_COMPONENTS})
				SET(OpenSG_ERROR_REASON
				"${OpenSG_ERROR_REASON}No OpenSG libraries were found. You may need to set OPENSG_LIBRARY_DIR to the directory containing OpenSG libraries or OPENSG_ROOT to the location of OpenSG.")
			ELSE(${__OpenSG_NUM_COMPONENTS_WANTED} EQUAL ${__OpenSG_NUM_MISSING_COMPONENTS})
				SET(OpenSG_ERROR_REASON
				"${OpenSG_ERROR_REASON}Some (but not all) of the required OpenSG libraries were found. You may need to install these additional OpenSG libraries. Alternatively, set OPENSG_LIBRARY_DIR to the directory containing OpenSG libraries or OPENSG_ROOT to the location of OpenSG.")
			ENDIF(${__OpenSG_NUM_COMPONENTS_WANTED} EQUAL ${__OpenSG_NUM_MISSING_COMPONENTS})
		ENDIF(__OpenSG_MISSING_COMPONENTS)


		# If OpenSG2 -> find Boost header and add to include DIRS
		IF(OpenSG_MAJOR_VERSION STREQUAL "2")
			# Find BOOST headers needed for OpenSG2
			if(WIN32) #on windows, static linking works better:
				set(Boost_USE_STATIC_LIBS ON)
			endif(WIN32)
			find_package(Boost COMPONENTS filesystem system REQUIRED QUIET)
			MESSAGE(STATUS "adding Boost_INCLUDE_DIR to OpenSG_INCLUDE_DIRS")
			SET(OpenSG_INCLUDE_DIRS ${OpenSG_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
			SET(OpenSG_LIBRARY_DIRS ${OpenSG_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
			set(OpenSG_LIBRARIES ${OpenSG_LIBRARIES} ${Boost_LIBRARIES})
			set(OpenSG_DEFINITIONS ${OpenSG_DEFINITIONS} ${Boost_DEFINITIONS})
		ENDIF(OpenSG_MAJOR_VERSION STREQUAL "2")
	ENDIF(OpenSG_INCLUDE_DIR)
	IF(OpenSG_FOUND)
		IF(NOT OpenSG_FIND_QUIETLY)
			MESSAGE(STATUS "OpenSG found.")
		ENDIF(NOT OpenSG_FIND_QUIETLY)

		IF (NOT OpenSG_FIND_QUIETLY)
			MESSAGE(STATUS "Found the following OpenSG libraries:")
		ENDIF(NOT OpenSG_FIND_QUIETLY)

		FOREACH(COMPONENT  ${OpenSG_FIND_RESOLVED_COMPONENTS})
			STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
			IF(OpenSG_${UPPERCOMPONENT}_FOUND)
				IF(NOT OpenSG_FIND_QUIETLY)
					MESSAGE(STATUS "  ${COMPONENT}")
				ENDIF(NOT OpenSG_FIND_QUIETLY)

				SETUP_OSG_LIB_TARGETS(${COMPONENT} ${UPPERCOMPONENT})

				SET(OpenSG_LIBRARIES ${OpenSG_LIBRARIES} ${OpenSG_${UPPERCOMPONENT}_LIBRARIES})
			ENDIF(OpenSG_${UPPERCOMPONENT}_FOUND)
		ENDFOREACH(COMPONENT)

	ELSE(OpenSG_FOUND)
		IF(OpenSG_FIND_REQUIRED)
			MESSAGE(SEND_ERROR "Unable to find the requested OpenSG libraries.\n${OpenSG_ERROR_REASON}")
		ENDIF(OpenSG_FIND_REQUIRED)
	ENDIF(OpenSG_FOUND)

ENDIF(__OpenSG_IN_CACHE)

set(OpenSG_DEFINITIONS_RELEASE ${OpenSG_DISABLE_MICROSOFT_SECURE_CXXX_DEFINITIONS} -D_OSG_HAVE_CONFIGURED_H_ -DOSG_WITH_GLUT -DOSG_WITH_PNG -DOSG_WITH_GIF -DOSG_WITH_TIF -DOSG_WITH_JPG)
set(OpenSG_DEFINITIONS_DEBUG   ${OpenSG_DEFINITIONS_RELEASE} -DOSG_DEBUG)
set(OpenSG_DEFINITIONS ${OpenSG_DEFINITIONS_RELEASE})
