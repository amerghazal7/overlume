/** @file tps_pybind.cpp @brief Python bindings (`tpscuda`) for parity testing. */
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "rendering_reprojector/reprojector.hpp"

namespace py = pybind11;
using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

static CameraParams to_cam(py::dict d)
{
    CameraParams c{};
    auto K = d["K"].cast<py::array_t<float>>();
    auto R = d["R"].cast<py::array_t<float>>();
    auto t = d["t"].cast<py::array_t<float>>();
    std::memcpy(c.K, K.data(), 9 * sizeof(float));
    std::memcpy(c.R, R.data(), 9 * sizeof(float));
    std::memcpy(c.t, t.data(), 3 * sizeof(float));
    c.width = d["width"].cast<int>();
    c.height = d["height"].cast<int>();
    return c;
}

PYBIND11_MODULE(tpscuda, m)
{
    py::class_<Reprojector>(m, "Reprojector")
        .def(py::init<int, int>())
        .def("set_cameras",
             [](Reprojector& r, std::vector<py::dict> cams) {
                 std::vector<CameraParams> cs;
                 for (auto& d : cams) cs.push_back(to_cam(d));
                 r.set_cameras(cs);
             })
        .def("upload_images",
             [](Reprojector& r, py::array_t<float, py::array::c_style | py::array::forcecast> a) {
                 r.upload_images(a.data(), a.shape(0), a.shape(1), a.shape(2));
             })
        .def("render_bowl",
             [](Reprojector& r, py::dict vcam, float R0, float k, float Rmax) {
                 CameraParams v = to_cam(vcam);
                 BowlParams b{R0, k, Rmax};
                 auto out = py::array_t<float>({v.height, v.width, 4});
                 r.render_bowl(v, b, out.mutable_data());
                 return out;
             });
}
