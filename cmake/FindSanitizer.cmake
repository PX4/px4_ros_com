# Turns on the available sanitizers for GCC: address and thread
#
# To activate, add the -DSANITIZER=address (or thread) options

set(SANITIZER_COMPILE_OPTIONS "-fno-omit-frame-pointer -fno-optimize-sibling-calls")

string(TOLOWER ${SANITIZER} SANITIZER)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.7)
  set(GCC_SANITIZERS address thread)
else()
  message(SEND_ERROR "Cannot use sanitizers without gcc >= 4.7")
endif()

if(${SANITIZER} IN_LIST GCC_SANITIZERS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZER_COMPILE_OPTIONS} -fsanitize=${SANITIZER}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=${SANITIZER}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=${SANITIZER}")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=${SANITIZER}")
  set(SANITIZER_FOUND TRUE)
  message(STATUS "${SANITIZER} sanitizer set for ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} compiler")
else()
  message(WARNING "GCC sanitizer '${SANITIZER}' not set, check compiler support")
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Sanitizer FOUND_VAR Sanitizer_FOUND
                                  REQUIRED_VARS SANITIZER_FOUND
                                 )
