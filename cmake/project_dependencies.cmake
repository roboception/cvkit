# Provides function find_package_public
# and variable RC_PUBLIC_BUILD_DEPENDENCIES.
#
# find_package_public works exactly the same way as find_package, only
# that for each call, the arguments are appended to
# RC_PUBLIC_BUILD_DEPENDENCIES, which forms a list of public build dependencies.
#
# E.g., for the call
# find_package_public(OpenCV 3 REQUIRED COMPONENTS core imgcodecs)
# the following string appended to the RC_PUBLIC_BUILD_DEPENDENCIES:
# "find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs)"
#
# RC_PUBLIC_BUILD_DEPENDENCIES can later be used in the PROJECTConfig.cmake.in
# file to automatically add find_package directives for the public dependencies.

set(RC_PUBLIC_BUILD_DEPENDENCIES "")
mark_as_advanced(RC_PUBLIC_BUILD_DEPENDENCIES)

macro(find_package_public)
  find_package(${ARGV})
  string (REPLACE ";" " " FIND_PACKAGE_PUBLIC_ARGV_STR "${ARGV}")
  list(APPEND RC_PUBLIC_BUILD_DEPENDENCIES "find_package(${FIND_PACKAGE_PUBLIC_ARGV_STR})")
endmacro()
