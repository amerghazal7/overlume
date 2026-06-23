message("############# tools CMake ##################")

# bench_reprojector — built unconditionally (no GL/window/ROS dep)
add_executable(bench_reprojector benchmark.cpp)
target_link_libraries(bench_reprojector PRIVATE rendering_reprojector)
target_include_directories(bench_reprojector PRIVATE
    ${CMAKE_SOURCE_DIR}/src/libs/rendering_reprojector/include)
