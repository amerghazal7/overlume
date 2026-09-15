# x64-linux-clang-libcxx.vcpkg triplet (VM-061) — every cesium-native vcpkg
# port on this platform is built by vcpkg with ITS OWN triplet, not with the
# enclosing project's CMAKE_CXX_FLAGS, so a stock x64-linux port set would
# be gcc/libstdc++ under this box's default vcpkg bootstrap toolchain (the
# yaml-cpp/GoogleTest precedent this whole file exists to avoid,
# CMakeLists.txt:77-93,240-261). This overlay triplet forces the same
# clang++ + -stdlib=libc++ the parent project enforces via
# VCPKG_CHAINLOAD_TOOLCHAIN_FILE, static linkage per Decision 2.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../vcpkg-clang-libcxx-toolchain.cmake")
