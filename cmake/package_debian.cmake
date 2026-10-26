# general cpack variables
set(CPACK_PACKAGE_CONTACT "Roboception GmbH <support@roboception.de>")
set(CPACK_PACKAGE_VENDOR "Roboception GmbH, Munich, Germany")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Roboception ${PROJECT_NAME} package")

# CPACK_PACKAGE_VERSION
if (RC_PACKAGE_VERSION)
    set(CPACK_PACKAGE_VERSION ${RC_PACKAGE_VERSION})
else ()
    message(WARNING "RC_PACKAGE_VERSION not set! Did you include project_version.cmake?")
    if (RC_PROJECT_VERSION)
        message(WARNING "CPACK_PACKAGE_VERSION: Falling back to RC_PROJECT_VERSION (${RC_PROJECT_VERSION})")
        set(CPACK_PACKAGE_VERSION ${RC_PROJECT_VERSION})
    elseif (PROJECT_VERSION)
        message(WARNING "CPACK_PACKAGE_VERSION: Falling back to PROJECT_VERSION (${PROJECT_VERSION})")
        set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
    endif ()
endif ()

# add date stamp to CPACK_PACKAGE_VERSION
string(TIMESTAMP STAMP "%Y%m%d+%H%M%S")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}-0+${STAMP}")


###############################
# debian package specific stuff
###############################
set(CPACK_GENERATOR "DEB")
#set(CPACK_DEBIAN_PACKAGE_DEBUG ON)

if (NOT CPACK_DEBIAN_PACKAGE_ARCHITECTURE)
# if architecture is already set (e.g. to "all"), this is not needed
# add ~distribution-codename (e.g. ~trusty or ~xenial) to end of package version
# if lsb_release is available, take it from there or fall back to DISTRO_CODENAME env variable
    set(DISTRO_CODENAME $ENV{DISTRO_CODENAME})
    find_program(LSB_RELEASE_CMD lsb_release)
    mark_as_advanced(LSB_RELEASE_CMD)
    if (LSB_RELEASE_CMD)
        execute_process(COMMAND "${LSB_RELEASE_CMD}" --codename --short
                OUTPUT_VARIABLE DISTRO_CODENAME
                OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif ()
    if (DISTRO_CODENAME)
        set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}~${DISTRO_CODENAME})
    else ()
        message(STATUS "Could not find lsb_release nor is DISTRO_CODENAME set.")
    endif ()

    find_program(DPKG_CMD dpkg)
    mark_as_advanced(DPKG_CMD)
    if (NOT DPKG_CMD)
        message(STATUS "Can not find dpkg in your path, default to i386.")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
    else ()
        execute_process(COMMAND "${DPKG_CMD}" --print-architecture
                OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
                OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif ()
endif ()
message(STATUS "CPACK_PACKAGE_VERSION: " ${CPACK_PACKAGE_VERSION})

# package name defaults to lower case of project name with _ replaced by -
if (NOT CPACK_PACKAGE_NAME)
    string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)
    string(REPLACE "_" "-" CPACK_PACKAGE_NAME "${PROJECT_NAME_LOWER}")
endif ()
message(STATUS "CPACK_PACKAGE_NAME: " ${CPACK_PACKAGE_NAME})

# check if it is a ROS/catkin package
if (EXISTS "${PROJECT_SOURCE_DIR}/package.xml")
    set(ROS_DISTRO $ENV{ROS_DISTRO})
    if (ROS_DISTRO)
        set(CPACK_PACKAGE_NAME "ros-${ROS_DISTRO}-${CPACK_PACKAGE_NAME}")
        # tell CPack to use CMAKE_INSTALL_PREFIX
        # cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" ..
        set(CPACK_SET_DESTDIR true)
    else ()
        message(STATUS "ROS_DISTRO not set. Not treating this as a ROS package.")
    endif ()
endif ()

if(EXCLUSIVE_CUSTOMER)
#use original name (without customer suffix) to declare the conflict
  set(CPACK_DEBIAN_PACKAGE_CONFLICTS ${CPACK_PACKAGE_NAME})
  set(CPACK_DEBIAN_PACKAGE_PROVIDES  ${CPACK_PACKAGE_NAME})
  set(CPACK_DEBIAN_PACKAGE_REPLACES  ${CPACK_PACKAGE_NAME})
#use a name with the name of the customer suffixed to build the package
  string(TOLOWER "${EXCLUSIVE_CUSTOMER}" CUSTOMER_NAME_LOWER)
  string(REPLACE "_" "-" CUSTOMER_SUFFIX "${CUSTOMER_NAME_LOWER}")
  set(CPACK_PACKAGE_NAME "${CPACK_PACKAGE_NAME}-${CUSTOMER_SUFFIX}")
endif()

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
message(STATUS "CPACK_PACKAGE_FILE_NAME: " ${CPACK_PACKAGE_FILE_NAME})

# The maintainer is used as supplier in SBOMs, which must be the company and
# not an individual. Therefore, it cannot be overwritten by the project.
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")

if (NOT CPACK_DEBIAN_PACKAGE_HOMEPAGE)
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://www.roboception.com")
endif ()

###############################################
# copyright file in machine readable format
# (https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/)
###############################################

# License as SPDX identifier, which can be set by the project before including
# this file. All licenses other than the default require RC_PACKAGE_LICENSE_FILE
# with the license text, e.g. for an open source project:
# set(RC_PACKAGE_LICENSE "BSD-3-Clause")
# set(RC_PACKAGE_LICENSE_FILE "${PROJECT_SOURCE_DIR}/LICENSE")
if (NOT RC_PACKAGE_LICENSE)
    set(RC_PACKAGE_LICENSE "LicenseRef-Roboception")
endif ()

if (RC_PACKAGE_LICENSE_FILE)
    file(READ "${RC_PACKAGE_LICENSE_FILE}" RC_LICENSE_TEXT)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${RC_PACKAGE_LICENSE_FILE}")
elseif (RC_PACKAGE_LICENSE STREQUAL "LicenseRef-Roboception")
    set(RC_LICENSE_TEXT
"All rights reserved.
This software is proprietary to Roboception GmbH. Use, copying and
distribution are only permitted under the terms of a license agreement
with Roboception GmbH.")
else ()
    message(FATAL_ERROR "RC_PACKAGE_LICENSE_FILE must be given for license ${RC_PACKAGE_LICENSE}")
endif ()

# The license text must be indented by one space and empty lines must be
# replaced by a single dot, otherwise the copyright file cannot be parsed.
string(STRIP "${RC_LICENSE_TEXT}" RC_LICENSE_TEXT)
string(REGEX REPLACE "[ \t\r]*\n" "\n" RC_LICENSE_TEXT "${RC_LICENSE_TEXT}")
string(REPLACE "\n\n" "\n.\n" RC_LICENSE_TEXT "${RC_LICENSE_TEXT}")
string(REPLACE "\n\n" "\n.\n" RC_LICENSE_TEXT "${RC_LICENSE_TEXT}")
string(REPLACE "\n" "\n " RC_LICENSE_TEXT "${RC_LICENSE_TEXT}")

# copyright years default to the years of the first commit and of the build
if (NOT RC_COPYRIGHT_YEARS)
    string(TIMESTAMP RC_BUILD_YEAR "%Y")
    set(RC_COPYRIGHT_YEARS ${RC_BUILD_YEAR})

    find_program(GIT_CMD git)
    mark_as_advanced(GIT_CMD)
    if (GIT_CMD)
        execute_process(COMMAND ${GIT_CMD} log --max-parents=0 --format=%ad --date=format:%Y HEAD
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                OUTPUT_VARIABLE RC_FIRST_YEARS
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
        string(REGEX MATCHALL "[0-9]+" RC_FIRST_YEARS "${RC_FIRST_YEARS}")
        if (RC_FIRST_YEARS)
            list(SORT RC_FIRST_YEARS)
            list(GET RC_FIRST_YEARS 0 RC_FIRST_YEAR)
            if (RC_FIRST_YEAR LESS RC_BUILD_YEAR)
                set(RC_COPYRIGHT_YEARS "${RC_FIRST_YEAR}-${RC_BUILD_YEAR}")
            endif ()
        endif ()
    endif ()
endif ()

# Copyright holders including years, which can be set by the project if
# Roboception GmbH is not the only holder, e.g.:
# set(RC_COPYRIGHT_HOLDERS "2016-2026 Roboception GmbH"
#   "2014, 2015 Institute of Robotics and Mechatronics, German Aerospace Center")
if (NOT RC_COPYRIGHT_HOLDERS)
    set(RC_COPYRIGHT_HOLDERS "${RC_COPYRIGHT_YEARS} Roboception GmbH")
endif ()

# multiple holders must be given as continuation lines, i.e. indented
string(REPLACE ";" "\n " RC_COPYRIGHT_TEXT "${RC_COPYRIGHT_HOLDERS}")

# Third party code that is contained in the package must be described by
# additional Files paragraphs in a file that can be set by the project, e.g.:
# set(RC_COPYRIGHT_EXTRA_FILE "${PROJECT_SOURCE_DIR}/copyright_extra")
set(RC_COPYRIGHT_EXTRA "")
if (RC_COPYRIGHT_EXTRA_FILE)
    file(READ "${RC_COPYRIGHT_EXTRA_FILE}" RC_COPYRIGHT_EXTRA)
    string(STRIP "${RC_COPYRIGHT_EXTRA}" RC_COPYRIGHT_EXTRA)
    set(RC_COPYRIGHT_EXTRA "${RC_COPYRIGHT_EXTRA}\n\n")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${RC_COPYRIGHT_EXTRA_FILE}")
endif ()

set(COPYRIGHT_FILE "${CMAKE_CURRENT_BINARY_DIR}/copyright")
file(WRITE "${COPYRIGHT_FILE}"
"Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: ${PROJECT_NAME}
Upstream-Contact: ${CPACK_PACKAGE_CONTACT}

Files: *
Copyright: ${RC_COPYRIGHT_TEXT}
License: ${RC_PACKAGE_LICENSE}

${RC_COPYRIGHT_EXTRA}License: ${RC_PACKAGE_LICENSE}
 ${RC_LICENSE_TEXT}
")

# absolute path, since the install prefix may be e.g. /opt/ros/<distro>
install(FILES "${COPYRIGHT_FILE}"
        COMPONENT bin
        DESTINATION /usr/share/doc/${CPACK_PACKAGE_NAME})

###############################################
# SBOM fragment, see rc_sbom_tools
###############################################

# The fragment describes the third party code in the package. It is created
# by rc-sbom-tools at the end of the installation, therefore this file must be
# included after all subdirectories have been added.
if (EXISTS "${PROJECT_SOURCE_DIR}/sbom-components.yaml")
    find_file(RC_SBOM_CMAKE rc_sbom.cmake PATHS /usr/share/rc_sbom_tools NO_DEFAULT_PATH)
    mark_as_advanced(RC_SBOM_CMAKE)
    if (RC_SBOM_CMAKE)
        include(${RC_SBOM_CMAKE})
    elseif (DEFINED ENV{CI})
        message(FATAL_ERROR "rc-sbom-tools must be installed for creating the SBOM fragment of the package")
    else ()
        message(WARNING "rc-sbom-tools not installed, the package will not contain an SBOM fragment")
    endif ()
endif ()

#########################################
## things you might need to change ??? ##
#########################################
# if this package doesn't provide any libs or binaries, add set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) in your main CMakeLists.txt
if (NOT DEFINED CPACK_DEBIAN_PACKAGE_SHLIBDEPS)
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
endif ()

# ??? this defaults to PROJECT_LIBRARIES which should be defined in main CMakeLists.txt before including this file
# list of shared library this package provides (; separated, comment or empty list if there are none)
# IMPORTANT: also the lib needs to set SOVERSION via set_target_properties, e.g.:
# set_target_properties(rcimage PROPERTIES SOVERSION ${abiversion})
if (PROJECT_LIBRARIES)
    set(sharedlibs ${PROJECT_LIBRARIES})
endif ()

# if there are shared libs exported by this package:
# generate debian shlibs file and trigger ldconfig
if (sharedlibs)
    set(SHLIBS_FILE "${CMAKE_CURRENT_BINARY_DIR}/shlibs")
    set(TRIGGERS_FILE "${CMAKE_CURRENT_BINARY_DIR}/triggers")

    # Generate triggers file
    file(WRITE "${TRIGGERS_FILE}" "activate-noawait ldconfig\n")

    # Generate shlibs file
    # also the lib needs to set SOVERSION via set_target_properties:
    # set_target_properties(rcimage PROPERTIES SOVERSION ${abiversion})
    file(WRITE "${SHLIBS_FILE}" "")
    foreach (libname ${sharedlibs})
        get_target_property(so_abiversion ${libname} SOVERSION)
        if(NOT ${so_abiversion})
          set(so_abiversion ${abiversion})
          message(STATUS "SOVERSION of shared lib \"${libname}\" not set explicitly. Using <Major.Minor> of latest tag: ${so_abiversion}")
          set_target_properties(${libname} PROPERTIES SOVERSION ${so_abiversion})
        endif()
        version_bump(${so_abiversion} so_abiversion_bumped)
        file(APPEND "${SHLIBS_FILE}" "lib${libname} ${so_abiversion} ${CPACK_PACKAGE_NAME} (>=${so_abiversion}), ${CPACK_PACKAGE_NAME} (<<${so_abiversion_bumped})\n")
    endforeach (libname)

    execute_process(COMMAND chmod 644 "${SHLIBS_FILE}" "${TRIGGERS_FILE}")
    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${SHLIBS_FILE};${TRIGGERS_FILE}")
endif ()

if (conffiles)
  set(CONFFILES_FILE "${CMAKE_CURRENT_BINARY_DIR}/conffiles")
  file(WRITE "${CONFFILES_FILE}" "")
  foreach (conffile ${conffiles})
    file(APPEND "${CONFFILES_FILE}" "${conffile}\n")
  endforeach (conffile)

  execute_process(COMMAND chmod 644 "${CONFFILES_FILE}")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA};${CONFFILES_FILE}")
endif ()

include(CPack)
