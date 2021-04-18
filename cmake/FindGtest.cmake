# message("-- Check for gtest")

# unset(GTEST_FOUND)
# unset(GTEST_INCLUDE_DIRS)
# unset(GTEST_LIBRARIES)

# include(FindPackageHandleStandardArgs)

# list(APPEND GTEST_CHECK_INCLUDE_DIRS
#     /usr/local/include
#     /usr/local/homebrew/include
#     /opt/local/var/macports/software
#     /opt/local/include
#     /usr/include)
# list(APPEND GTEST_CHECK_PATH_SUFFIXES
#     gtest/include
#     gtest/Include
#     Glog/include
#     Glog/Include
#     src/windows)

# list(APPEND GTEST_CHECK_LIBRARY_DIRS
#     /usr/local/lib
#     /usr/local/homebrew/lib
#     /opt/local/lib
#     /usr/lib)
# list(APPEND GTEST_CHECK_LIBRARY_SUFFIXES
#     gtest/lib
#     gtest/Lib
#     Glog/lib
#     Glog/Lib
#     x64/Release)

# find_path(GTEST_INCLUDE_DIRS
#   NAMES gtest/gtest.h
#   PATHS ${GTEST_CHECK_INCLUDE_DIRS}
#   PATH_SUFFIXES ${GTEST_CHECK_PATH_SUFFIXES})

# find_library(GTEST_LIBRARIES
#   NAMES gtest libgtest gtest_main libgtest_main
#   PATHS ${GTEST_CHECK_LIBRARY_DIRS}
#   PATH_SUFFIXES ${GTEST_CHECK_LIBRARY_SUFFIXES})

# if (GTEST_INCLUDE_DIRS AND GTEST_LIBRARIES)
#   set(GTEST_FOUND TRUE)
#   message(STATUS "Found Gtest")
#   message(STATUS "  Includes: ${GTEST_INCLUDE_DIRS}")
#   message(STATUS "  Libraries: ${GTEST_LIBRARIES}")
# else()
#   if (Gtest_FIND_REQUIRED)
#     message(FATAL_ERROR "Could not find Gtest")
#   endif()
# endif()
