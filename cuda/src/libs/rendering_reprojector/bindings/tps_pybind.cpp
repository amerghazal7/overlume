/** @file tps_pybind.cpp @brief Python bindings (`tpscuda`) for parity testing. */
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <vector>

#include "rendering_reprojector/mesh_loader.hpp"
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
    if (d.contains("dist"))
    {
        auto dd = d["dist"].cast<py::array_t<float, py::array::c_style | py::array::forcecast>>();
        for (py::ssize_t i = 0; i < 5 && i < dd.size(); ++i) c.dist[i] = dd.data()[i];
    }
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
             [](Reprojector& r, py::dict vcam, float R0, float k, float Rmax,
                float feather_margin, bool fill_blind_zone) {
                 CameraParams v = to_cam(vcam);
                 BowlParams b{R0, k, Rmax, feather_margin, fill_blind_zone};
                 auto out = py::array_t<float>({v.height, v.width, 4});
                 r.render_bowl(v, b, out.mutable_data());
                 return out;
             },
             py::arg("vcam"), py::arg("R0"), py::arg("k"), py::arg("Rmax"),
             py::arg("feather_margin") = 30.0f, py::arg("fill_blind_zone") = false)
        .def("upload_depth",
             [](Reprojector& r,
                py::array_t<float, py::array::c_style | py::array::forcecast> a) {
                 r.upload_depth(a.data(), a.shape(0), a.shape(1), a.shape(2));
             })
        .def("upload_points",
             [](Reprojector& r,
                py::array_t<float, py::array::c_style | py::array::forcecast> pts,
                float feather_margin) {
                 if (pts.ndim() != 2 || pts.shape(1) != 3)
                     throw std::invalid_argument("upload_points expects (N,3) float xyz");
                 r.upload_points(pts.data(), static_cast<std::size_t>(pts.shape(0)),
                                 feather_margin);
             },
             py::arg("pts"), py::arg("feather_margin") = 30.0f)
        .def("upload_robot_mesh",
             [](Reprojector& r,
                py::array_t<float, py::array::c_style | py::array::forcecast> verts,
                py::array_t<float, py::array::c_style | py::array::forcecast> cols) {
                 if (verts.ndim() != 2 || verts.shape(1) != 9 || cols.ndim() != 2 ||
                     cols.shape(1) != 3 || verts.shape(0) != cols.shape(0))
                     throw std::invalid_argument(
                         "upload_robot_mesh expects verts (N,9) and cols (N,3)");
                 r.upload_robot_mesh(verts.data(), cols.data(),
                                     static_cast<std::size_t>(verts.shape(0)));
             })
        .def("upload_self_masks",
             [](Reprojector& r,
                py::array_t<unsigned char, py::array::c_style | py::array::forcecast> m) {
                 if (m.ndim() != 3)
                     throw std::invalid_argument("upload_self_masks expects (N,H,W) uint8");
                 r.upload_self_masks(m.data(), static_cast<int>(m.shape(0)),
                                     static_cast<int>(m.shape(1)),
                                     static_cast<int>(m.shape(2)));
             })
        .def("render_depth",
             [](Reprojector& r, py::dict vcam, int splat_radius) {
                 CameraParams v = to_cam(vcam);
                 auto out = py::array_t<float>({v.height, v.width, 4});
                 r.render_depth(v, splat_radius, out.mutable_data());
                 return out;
             })
        .def("render_hybrid",
             [](Reprojector& r, py::dict vcam, float R0, float k, float Rmax, int radius,
                float feather_margin, bool fill_blind_zone) {
                 CameraParams v = to_cam(vcam);
                 BowlParams b{R0, k, Rmax, feather_margin, fill_blind_zone};
                 auto out = py::array_t<float>({v.height, v.width, 4});
                 r.render_hybrid(v, b, radius, out.mutable_data());
                 return out;
             },
             py::arg("vcam"), py::arg("R0"), py::arg("k"), py::arg("Rmax"),
             py::arg("radius"), py::arg("feather_margin") = 30.0f,
             py::arg("fill_blind_zone") = false);

    m.def("load_obj_mesh",
          [](const std::string& path,
             py::array_t<float, py::array::c_style | py::array::forcecast> T) {
              if (T.size() != 12)
                  throw std::invalid_argument("transform must be 12 floats [R(9)|t(3)]");
              auto mesh = micropilot::rendering::load_obj_mesh(path, T.data());
              py::ssize_t n = static_cast<py::ssize_t>(mesh.n_tris);
              auto verts = py::array_t<float>({n, static_cast<py::ssize_t>(9)});
              auto cols = py::array_t<float>({n, static_cast<py::ssize_t>(3)});
              std::memcpy(verts.mutable_data(), mesh.verts.data(),
                          mesh.verts.size() * sizeof(float));
              std::memcpy(cols.mutable_data(), mesh.cols.data(),
                          mesh.cols.size() * sizeof(float));
              return py::make_tuple(verts, cols);
          },
          py::arg("path"), py::arg("transform"));
}
