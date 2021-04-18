set(GFLAGS_INCLUDE_DIR_HINTS "" CACHE PATH "Glog include directory")
set(GFLAGS_LIBRARY_DIR_HINTS "" CACHE PATH "Glog library directory")

unset(GFLAGS_FOUND)
unset(GFLAGS_INCLUDE_DIRS)
unset(GFLAGS_LIBRARIES)

include(FindPackageHandleStandardArgs)

list(APPEND GFLAGS_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include
    /opt/local/var/macports/software
    /opt/local/include
    /usr/include)
list(APPEND GFLAGS_CHECK_PATH_SUFFIXES
    gflags/include
    gflags/Include
    Gflags/include
    Gflags/Include
    src/windows)

list(APPEND GFLAGS_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib
    /opt/local/lib
    /usr/lib)
list(APPEND GFLAGS_CHECK_LIBRARY_SUFFIXES
    gflags/lib
    gflags/Lib
    Gflags/lib
    Gflags/Lib
    x64/Release)

find_path(GFLAGS_INCLUDE_DIRS
    NAMES
    gflags/gflags.h
    PATHS
    ${GFLAGS_INCLUDE_DIR_HINTS}
    ${GFLAGS_CHECK_INCLUDE_DIRS}
    PATH_SUFFIXES
    ${GFLAGS_CHECK_PATH_SUFFIXES})
find_library(GFLAGS_LIBRARIES
    NAMES
    gflags
    libgflags
    PATHS
    ${GFLAGS_LIBRARY_DIR_HINTS}
    ${GFLAGS_CHECK_LIBRARY_DIRS}
    PATH_SUFFIXES
    ${GFLAGS_CHECK_LIBRARY_SUFFIXES})

if (GFLAGS_INCLUDE_DIRS AND GFLAGS_LIBRARIES)
  set(GFLAGS_FOUND TRUE)
  message(STATUS "Found GFlags")
  message(STATUS "  Includes : ${GFLAGS_INCLUDE_DIRS}")
  message(STATUS "  Libraries : ${GFLAGS_LIBRARIES}")
else()
  if(GFlags_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find Glog")
  endif()
endif()
