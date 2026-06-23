cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME "micropilot_rendering")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_ENABLE_EXPORTS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "CUDA arch (86=RTX3090; set 89 for Ada deploy)")

set(MPR_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install/libs")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${MPR_INSTALL_PREFIX}/rendering_reprojector/libs")

project("${PROJECT_NAME}" VERSION 0.1.0 DESCRIPTION "micropilot_rendering" LANGUAGES CXX CUDA)

set(CMAKE_INSTALL_PREFIX "${MPR_INSTALL_PREFIX}" CACHE PATH "" FORCE)
option(BUILD_TESTING "Build unit tests" OFF)
option(BUILD_PYBIND "Build the tpscuda pybind11 module" ON)

if(BUILD_TESTING)
    enable_testing()
endif()

add_subdirectory(src/libs)

include(CMakePackageConfigHelpers)
install(EXPORT micropilot_renderingTargets
    NAMESPACE micropilot_rendering::
    DESTINATION lib/cmake/micropilot_rendering)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/micropilot_renderingConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfig.cmake"
    INSTALL_DESTINATION lib/cmake/micropilot_rendering)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfigVersion.cmake"
    DESTINATION lib/cmake/micropilot_rendering)
