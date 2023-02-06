# This cmake code creates the configuration that is found and used by
# find_package() of another cmake project

# Do not Error on non-existent target in get_target_property.
if (POLICY CMP0045)
  cmake_policy(SET CMP0045 OLD)
endif ()


if (NOT PROJECT_NAME_UPPER)
  string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
endif ()
if (NOT PROJECT_NAME_LOWER)
  string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)
endif ()
if (NOT PROJECT_NAMESPACE)
  set(PROJECT_NAMESPACE "${PROJECT_NAME}")
endif ()

if (NOT RC_PACKAGE_VERSION)
  message(FATAL_ERROR "RC_PACKAGE_VERSION not set! include project_version.cmake")
endif ()


# Go through all static libraries and add the INTERFACE_LINK_LIBRARIES of all
# dependencies also to the INTERFACE_LINK_LIBRARIES of the static libraries

foreach (LIB ${PROJECT_STATIC_LIBRARIES})
  get_target_property(LIB_DEPS ${LIB} INTERFACE_LINK_LIBRARIES)

  if (LIB_DEPS)
    set(ADD_LIB_DEBS)
    foreach (DEP ${LIB_DEPS})
      get_target_property(DEP_DEP ${DEP} INTERFACE_LINK_LIBRARIES)

      if (NOT DEP_DEP)
        get_target_property(DEP_DEP ${DEP} IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE)
      endif ()

      if (DEP_DEP)
        set(ADD_LIB_DEPS ${ADD_LIB_DEPS} ${DEP_DEP})
      endif ()
    endforeach ()

    set(LIB_DEPS ${LIB_DEPS} ${ADD_LIB_DEPS})
    set_target_properties(${LIB} PROPERTIES INTERFACE_LINK_LIBRARIES "${LIB_DEPS}")
  endif ()
endforeach ()

# configure and install the configuration files

include(CMakePackageConfigHelpers)

string(REPLACE ";" "\n" RC_PUBLIC_BUILD_DEPENDENCIES_STR "${RC_PUBLIC_BUILD_DEPENDENCIES}")
mark_as_advanced(RC_PUBLIC_BUILD_DEPENDENCIES_STR)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/PROJECTConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_UPPER}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME_LOWER}
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR RC_PUBLIC_BUILD_DEPENDENCIES_STR)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_UPPER}ConfigVersion.cmake
    VERSION ${RC_PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_UPPER}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_UPPER}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME_LOWER}
    COMPONENT dev)

if (PROJECT_LIBRARIES OR PROJECT_STATIC_LIBRARIES)
  install(EXPORT PROJECTTargets
      NAMESPACE ${PROJECT_NAMESPACE}::
      FILE ${PROJECT_NAME_UPPER}Targets.cmake
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME_LOWER}
      COMPONENT dev)
endif ()
