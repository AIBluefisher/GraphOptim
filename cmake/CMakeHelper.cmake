if(POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)
endif()

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

# Determine project compiler.
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(IS_MSVC TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(IS_GNU TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(IS_CLANG TRUE)
endif()

# Determine project architecture.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "[ix].?86|amd64|AMD64")
    set(IS_X86 TRUE)
endif()

# Determine project operating system.
string(REGEX MATCH "Linux" IS_LINUX ${CMAKE_SYSTEM_NAME})
string(REGEX MATCH "DragonFly|BSD" IS_BSD ${CMAKE_SYSTEM_NAME})
string(REGEX MATCH "SunOS" IS_SOLARIS ${CMAKE_SYSTEM_NAME})
if(WIN32)
    SET(IS_WINDOWS TRUE BOOL INTERNAL)
endif()
if(APPLE)
    SET(IS_MACOS TRUE BOOL INTERNAL)
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
if(CMAKE_BUILD_TYPE_LOWER STREQUAL "debug"
   OR CMAKE_BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
    set(IS_DEBUG TRUE)
endif()

set(GRAPH_OPTIMIZER_LIB "gopt")

# Macro to add source files to GRAPH_OPTIMIZER library.
macro(OPTIMIZER_ADD_SOURCES)
  set(SOURCE_FILES "")
  foreach(SOURCE_FILE ${ARGN})
    if(SOURCE_FILE MATCHES "^/.*")
      list(APPEND SOURCE_FILES ${SOURCE_FILE})
    else()
      list(APPEND SOURCE_FILES
           "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FILE}")
    endif()
  endforeach()
  set(GRAPH_OPTIMIZER_SOURCES
      ${GRAPH_OPTIMIZER_SOURCES} ${SOURCE_FILES} PARENT_SCOPE)
endmacro(OPTIMIZER_ADD_SOURCES)

# Macro to add header files to GRAPH_OPTIMIZER library.
macro(OPTIMIZER_ADD_HEADERS)
  set(SOURCE_FILES "")
  foreach(SOURCE_FILE ${ARGN})
    if(SOURCE_FILE MATCHES "^/.*")
        list(APPEND SOURCE_FILES ${SOURCE_FILE})
    else()
        list(APPEND SOURCE_FILES
             "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FILE}")
    endif()
  endforeach()
  set(GRAPH_OPTIMIZER_HEADERS
      ${GRAPH_OPTIMIZER_HEADERS} ${SOURCE_FILES} PARENT_SCOPE)
endmacro(OPTIMIZER_ADD_HEADERS)

macro(OPTIMIZER_ADD_GTEST TEST_TARGET_NAME TEST_SOURCE_FILE)
  # set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/test)

  add_executable(${TEST_TARGET_NAME} ${TEST_SOURCE_FILE})
  target_link_libraries(${TEST_TARGET_NAME}
    ${GRAPH_OPTIMIZER_EXTERNAL_LIBRARIES} ${GRAPH_OPTIMIZER_LIB})
  add_test(${TEST_TARGET_NAME} ${EXECUTATLE_OUTPUT_PATH}/${TEST_TARGET_NAME})
endmacro(OPTIMIZER_ADD_GTEST)

macro(OPTIMIZER_ADD_EXE TARGET_NAME SOURCE_FILE)
  # set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/test)
  add_executable(${TARGET_NAME} ${SOURCE_FILE})
  target_link_libraries(${TARGET_NAME}
    ${GRAPH_OPTIMIZER_EXTERNAL_LIBRARIES} ${GRAPH_OPTIMIZER_LIB})
endmacro(OPTIMIZER_ADD_EXE)

# Replacement for the normal add_library() command. The syntax remains the same
# in that the first argument is the target name, and the following arguments
# are the source files to use when building the target.
macro(OPTIMIZER_ADD_LIBRARY TARGET_NAME)
    # ${ARGN} will store the list of source files passed to this function.
    add_library(${TARGET_NAME} ${ARGN})
    # set_target_properties(${TARGET_NAME} PROPERTIES FOLDER
    #     ${OPTIMIZER_TARGETS_ROOT_FOLDER}/${FOLDER_NAME})
    # install(TARGETS ${TARGET_NAME} DESTINATION 3rd_party/gopt)
endmacro(OPTIMIZER_ADD_LIBRARY)
