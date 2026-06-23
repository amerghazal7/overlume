message("############# rendering_reprojector Lib CMake ##################")

find_package(CUDAToolkit REQUIRED)

add_library(rendering_reprojector SHARED
    src/reprojector.cpp
    src/kernels/fill.cu
)
set_target_properties(rendering_reprojector PROPERTIES EXPORT_NAME reprojector)

target_include_directories(rendering_reprojector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:rendering_reprojector/include/rendering_reprojector>
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
    else()
        message(STATUS "GTest not found; skipping rendering_reprojector tests.")
    endif()
endif()
