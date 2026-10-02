# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# CPack configuration. Included last from overlume/CMakeLists.txt, after the
# install rules in OverlumeInstall.cmake (components "overlume" and "static").
set(CPACK_PACKAGE_NAME overlume)
set(CPACK_PACKAGE_VENDOR "Amer Ghazal")
set(CPACK_PACKAGE_CONTACT "Amer Ghazal <amer.ghazal@micropolis.ae>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Overlume real-time robot-scene rendering library")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/amerghazal7/overlume")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE")
string(TOLOWER "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}" _ov_plat)
set(CPACK_PACKAGE_FILE_NAME "overlume-${PROJECT_VERSION}-${_ov_plat}")
set(CPACK_STRIP_FILES ON)
set(CPACK_COMPONENTS_ALL overlume static)
set(CPACK_COMPONENT_STATIC_DEPENDS overlume)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CPACK_GENERATOR "DEB;RPM;TGZ")
    set(CPACK_PACKAGING_INSTALL_PREFIX /usr)
    set(CPACK_DEB_COMPONENT_INSTALL ON)
    set(CPACK_DEBIAN_OVERLUME_PACKAGE_NAME overlume)
    set(CPACK_DEBIAN_STATIC_PACKAGE_NAME overlume-static)
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    # ponytail: explicit Depends. dpkg-shlibdeps cannot resolve anything in the Alma build
    # container (no dpkg database); the .so's only external NEEDED are libc and EGL/GLESv2.
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
    set(CPACK_DEBIAN_STATIC_PACKAGE_DEPENDS "libegl-dev, libgles-dev")  # -lEGL -lGLESv2 in the static link line
    set(CPACK_DEBIAN_OVERLUME_PACKAGE_DEPENDS "libc6 (>= 2.28), libegl1, libgles2, libgl1")  # libGL.so.1 is dlopen'ed by Filament's bluegl
    set(CPACK_DEBIAN_ENABLE_COMPONENT_DEPENDS ON)
    set(CPACK_DEBIAN_PACKAGE_SECTION libs)
    set(CPACK_RPM_COMPONENT_INSTALL ON)
    set(CPACK_RPM_OVERLUME_PACKAGE_NAME overlume)
    set(CPACK_RPM_STATIC_PACKAGE_NAME overlume-static)
    set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
    set(CPACK_RPM_PACKAGE_LICENSE "Apache-2.0")
    set(CPACK_RPM_PACKAGE_AUTOREQ ON)
    # The static link line carries -lEGL -lGLESv2: the consumer needs the unversioned .so links.
    set(CPACK_RPM_STATIC_PACKAGE_REQUIRES "overlume = ${PROJECT_VERSION}, mesa-libEGL-devel, libglvnd-devel")
    set(CPACK_RPM_OVERLUME_PACKAGE_REQUIRES "libGL.so.1()(64bit)")  # dlopen'ed by Filament's bluegl
    set(CPACK_RPM_OVERLUME_POST_INSTALL_SCRIPT_FILE "${CMAKE_CURRENT_LIST_DIR}/rpm/post.sh")
    set(CPACK_RPM_OVERLUME_POST_UNINSTALL_SCRIPT_FILE "${CMAKE_CURRENT_LIST_DIR}/rpm/postun.sh")
    set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION
        /usr/lib/cmake /usr/share/pkgconfig /usr/share/doc)
    set(CPACK_ARCHIVE_COMPONENT_INSTALL OFF)  # one tar.gz with both components
endif()
# The deb/rpm payloads install under /usr; the tar.gz is prefix-relative so it can be unpacked anywhere.
configure_file("${CMAKE_CURRENT_LIST_DIR}/OverlumeCPackOptions.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/OverlumeCPackOptions.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/OverlumeCPackOptions.cmake")
# Tasks 6-8 append ANDROID / APPLE / WIN32 branches here.
include(CPack)
