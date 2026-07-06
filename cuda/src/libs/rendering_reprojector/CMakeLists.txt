message("############# rendering_reprojector Lib CMake ##################")

find_package(CUDAToolkit REQUIRED)

add_library(rendering_reprojector SHARED
    src/reprojector.cpp
    src/mesh_loader.cpp
    src/kernels/fill.cu
    src/kernels/reproject.cu
    src/kernels/splat.cu
    src/kernels/robot_raster.cu
)
set_target_properties(rendering_reprojector PROPERTIES EXPORT_NAME reprojector)

target_include_directories(rendering_reprojector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:rendering_reprojector/include/rendering_reprojector>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(rendering_reprojector PUBLIC CUDA::cudart)

install(TARGETS rendering_reprojector
        EXPORT micropilot_renderingTargets
        LIBRARY DESTINATION rendering_reprojector/libs)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION rendering_reprojector/include/rendering_reprojector)

if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(test_reprojector tests/test_reprojector.cpp)
        target_link_libraries(test_reprojector PRIVATE rendering_reprojector GTest::GTest GTest::Main)
        include(GoogleTest)
        gtest_discover_tests(test_reprojector DISCOVERY_TIMEOUT 30)

        # Golden test: reads cuda/tests/golden/ and checks PSNR > 40 dB vs NumpyRenderer.
        add_executable(test_golden tests/test_golden.cpp)
        target_link_libraries(test_golden PRIVATE rendering_reprojector GTest::GTest GTest::Main)
        # GOLDEN_DIR points to the fixture directory.
        # CMAKE_SOURCE_DIR is cuda/ (the top-level CMakeLists.txt).
        target_compile_definitions(test_golden PRIVATE
            GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/golden")
        gtest_discover_tests(test_golden DISCOVERY_TIMEOUT 30)
    else()
        message(STATUS "GTest not found; skipping rendering_reprojector tests.")
    endif()
endif()
