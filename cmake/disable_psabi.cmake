# Disable verbose notes from https://gcc.gnu.org/bugzilla/show_bug.cgi?id=77728 on arm

if (UNIX)
  # get architecture from compiler
  EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine COMMAND tr -d '\n' OUTPUT_VARIABLE CXX_MACHINE)
  string(REGEX REPLACE "([a-zA-Z_0-9]+).*" "\\1" ARCHITECTURE ${CXX_MACHINE})

  if ("${ARCHITECTURE}" STREQUAL "arm")
      message(STATUS "Disabled psABI warnings")
      add_definitions(-Wno-psabi)
  endif()

endif()
