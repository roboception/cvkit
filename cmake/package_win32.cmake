
# settings for NSIS (Windows) install package

set(CPACK_PACKAGE_CONTACT "Heiko Hirschmueller <heiko.hirschmueller@roboception.de>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Computer Vision Toolkit")
set(CPACK_PACKAGE_VENDOR "Institute of Robotics and Mechatronics, German Aerospace Center")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

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

set(CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION_PATCH})

# add date stamp to CPACK_PACKAGE_VERSION
string(TIMESTAMP STAMP "%Y%m%d+%H%M%S")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}-0+${STAMP}")

set(CPACK_GENERATOR NSIS)

set(CPACK_NSIS_INSTALL_ROOT "C:")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "cvkit")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\plyv.exe' '.ply' 'cvkit.plyv.ply'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.pfm' 'cvkit.sv.pfm'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.pgm' 'cvkit.sv.pgm'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.ppm' 'cvkit.sv.ppm'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.tif' 'cvkit.sv.tif'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.jpg' 'cvkit.sv.jpg'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.png' 'cvkit.sv.png'
  \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.vrt' 'cvkit.sv.vrt'
  System::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'
")

set(CPACK_NSIS_CREATE_ICONS_EXTRA "
  CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Example.lnk' '\\\$INSTDIR\\\\example'
  CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Readme.lnk' '\\\$INSTDIR\\\\doc\\\\README.TXT'
  CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Usage.lnk' '\\\$INSTDIR\\\\doc\\\\USAGE.TXT'
")

set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
  \\\${unregisterExtension} '.ply' 'cvkit.plyv.ply'
  \\\${unregisterExtension} '.pfm' 'cvkit.sv.pfm'
  \\\${unregisterExtension} '.pgm' 'cvkit.sv.pgm'
  \\\${unregisterExtension} '.ppm' 'cvkit.sv.ppm'
  \\\${unregisterExtension} '.tif' 'cvkit.sv.tif'
  \\\${unregisterExtension} '.jpg' 'cvkit.sv.jpg'
  \\\${unregisterExtension} '.png' 'cvkit.sv.png'
  \\\${unregisterExtension} '.vrt' 'cvkit.sv.vrt'
  System::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'
")

set(CPACK_NSIS_DELETE_ICONS_EXTRA "
  Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Readme.lnk'
  Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Usage.lnk'
  Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Example.lnk'
")

include(CPack)

