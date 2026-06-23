find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(tpscuda tps_pybind.cpp)
target_link_libraries(tpscuda PRIVATE rendering_reprojector)
install(TARGETS tpscuda LIBRARY DESTINATION rendering_reprojector/python)
