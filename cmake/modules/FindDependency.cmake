#general script to search for dependency libraries and  headers 
##############################################################################
#input variables
#DEP
#DEP_HEADER
#DEP_LIB
#custom paths
#DEP_INCLUDE_SEARCH_DIR}
#DEP_LIBRARY_SEARCH_DIR}
##############################################################################
#output variables
#DEP_INCLUDE_DIRS
#DEP_LIBRARY
MACRO(find_dependency DEP DEP_HEADER DEP_LIB)
##############################################################################
# search paths
##############################################################################
SET(${DEP}_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/${DEP}
  ${${DEP}_INCLUDE_DIRS}
  ${${DEP}_INCLUDE_SEARCH_DIR}
  "/usr/include"
  "/usr/share/include"
  "/usr/include/x86_64-linux-gnu/"
)

SET(${DEP}_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${${DEP}_LIBRARY_DIRS}
  ${${DEP}_LIBRARY_SEARCH_DIR}
  "/usr/lib/"
  "/usr/lib/x86_64-linux-gnu/"
)
##############################################################################
# feedback to provide user-defined paths to search for ${DEP}
##############################################################################
MACRO (request_dep_search_directories)

  IF ( NOT ${DEP}_INCLUDE_DIRS )
    SET(${DEP}_INCLUDE_SEARCH_DIR "Please provide ${DEP} include path." CACHE PATH "path to ${DEP} headers.")
    MESSAGE(FATAL_ERROR "find_${DEP}.cmake: unable to find ${DEP} headers.")
  ELSE ( NOT ${DEP}_INCLUDE_DIRS )
    UNSET(${DEP}_INCLUDE_SEARCH_DIR CACHE)
  ENDIF ( NOT ${DEP}_INCLUDE_DIRS )

  IF ( NOT ${DEP}_LIBRARY_DIRS )
    SET(${DEP}_LIBRARY_SEARCH_DIR "Please provide ${DEP} library path." CACHE PATH "path to ${DEP} libraries.")
    MESSAGE(FATAL_ERROR "find_${DEP}.cmake: unable to find ${DEP} libraries.")
  ELSE ( NOT ${DEP}_LIBRARY_DIRS )
    UNSET(${DEP}_LIBRARY_SEARCH_DIR CACHE)
  ENDIF ( NOT ${DEP}_LIBRARY_DIRS ) 

ENDMACRO (request_dep_search_directories)

##############################################################################
# search
##############################################################################
message(STATUS "Looking for ${DEP}")
# search for include directory
IF (NOT ${DEP}_INCLUDE_DIRS)

  SET(_${DEP}_FOUND_INC_DIRS "")
  FOREACH(_SEARCH_DIR ${${DEP}_INCLUDE_SEARCH_DIRS})
    FIND_PATH(_CUR_SEARCH
      NAMES ${DEP_HEADER}
        PATHS ${_SEARCH_DIR}
        NO_DEFAULT_PATH)
    IF (_CUR_SEARCH)
      LIST(APPEND _${DEP}_FOUND_INC_DIRS ${_CUR_SEARCH})
    ENDIF(_CUR_SEARCH)
    SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
  ENDFOREACH(_SEARCH_DIR ${${DEP}_INCLUDE_SEARCH_DIRS})

  IF (NOT _${DEP}_FOUND_INC_DIRS)
    request_dep_search_directories()
  ENDIF (NOT _${DEP}_FOUND_INC_DIRS)

  FOREACH(_INC_DIR ${_${DEP}_FOUND_INC_DIRS})
    SET(${DEP}_INCLUDE_DIRS ${${DEP}_INCLUDE_DIRS} ${_INC_DIR} CACHE PATH "${DEP} include directory.")
  ENDFOREACH(_INC_DIR ${_${DEP}_FOUND_INC_DIRS})

ENDIF (NOT ${DEP}_INCLUDE_DIRS)

IF(UNIX)
  SET(${DEP}_LIB_FILENAME "${DEP_LIB}.so")
ELSEIF(WIN32)
  SET(${DEP}_LIB_FILENAME "${DEP_LIB}.lib")
ENDIF(UNIX)
# search for library dirs
IF ( NOT ${DEP}_LIBRARY_DIRS )

  SET(_${DEP}_FOUND_LIB_DIR "")
  SET(_${DEP}_POSTFIX "")
  # check all given possible library dirs
  FOREACH(_SEARCH_DIR ${${DEP}_LIBRARY_SEARCH_DIRS})
    FIND_PATH(_CUR_SEARCH
      NAMES ${${DEP}_LIB_FILENAME}
        PATHS ${_SEARCH_DIR}
        PATH_SUFFIXES release debug
        NO_DEFAULT_PATH)
    # stop search once dir is found
    IF (_CUR_SEARCH)
      LIST(APPEND _${DEP}_FOUND_LIB_DIR ${_SEARCH_DIR})
      BREAK()
    ENDIF(_CUR_SEARCH)
    SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
  ENDFOREACH(_SEARCH_DIR ${${DEP}_LIBRARY_SEARCH_DIRS})

  # react to search result
  IF (NOT _${DEP}_FOUND_LIB_DIR)
    request_dep_search_directories()
  ELSE (NOT _${DEP}_FOUND_LIB_DIR)
    SET(${DEP}_LIBRARY_DIRS ${_${DEP}_FOUND_LIB_DIR} CACHE INTERNAL PATH "The ${DEP} library directory")
  ENDIF (NOT _${DEP}_FOUND_LIB_DIR)

  # accumulate library list
  LIST(APPEND _${DEP}_LIBRARIES "${_${DEP}_FOUND_LIB_DIR}/${${DEP}_LIB_FILENAME}")
  # Cannot use ARGN directly with list() command.
  # Copy to a variable first.
  SET(extra_macro_args ${ARGN})
  FOREACH(EXTRA_LIB ${extra_macro_args})
    LIST(APPEND _${DEP}_LIBRARIES "${_${DEP}_FOUND_LIB_DIR}/${EXTRA_LIB}")
  ENDFOREACH(EXTRA_LIB ${extra_macro_args})
  # set library cache variable
  IF (_${DEP}_FOUND_LIB_DIR)
    SET(${DEP}_LIBRARIES ${_${DEP}_LIBRARIES} CACHE FILEPATH "The ${DEP} library filename.")
  ENDIF (_${DEP}_FOUND_LIB_DIR)

ENDIF ( NOT ${DEP}_LIBRARY_DIRS )

##############################################################################
# verify
##############################################################################
IF ( NOT ${DEP}_INCLUDE_DIRS OR NOT ${DEP}_LIBRARY_DIRS )
  request_dep_search_directories()
  message(STATUS "Looking for ${DEP} - not found")
ELSE ( NOT ${DEP}_INCLUDE_DIRS OR NOT ${DEP}_LIBRARY_DIRS ) 
  UNSET(${DEP}_INCLUDE_SEARCH_DIR CACHE)
  UNSET(${DEP}_LIBRARY_SEARCH_DIR CACHE)
  message(STATUS "Looking for ${DEP} - found")
ENDIF ( NOT ${DEP}_INCLUDE_DIRS OR NOT ${DEP}_LIBRARY_DIRS )

ENDMACRO(find_dependency)