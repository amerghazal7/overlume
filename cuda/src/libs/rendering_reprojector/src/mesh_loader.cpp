/** @file mesh_loader.cpp @brief OBJ+MTL loader for the robot proxy mesh. */

#include "rendering_reprojector/mesh_loader.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace micropilot::rendering
{
namespace
{
struct V3
{
    float x = 0, y = 0, z = 0;
};

// Fixed light direction, rig frame (from above/front) — keep in sync with
// tests/test_robot_proxy.py::_L.
const V3 kLight = []{
    float n = std::sqrt(0.3f * 0.3f + 0.15f * 0.15f + 0.94f * 0.94f);
    return V3{0.3f / n, 0.15f / n, 0.94f / n};
}();

// Material accumulator: Kd and Ke tracked separately, folded on lookup.
struct Mat
{
    V3 kd{0.7f, 0.7f, 0.7f};
    V3 ke{0.0f, 0.0f, 0.0f};
};

V3 mat_color(const Mat& m)
{
    // emissive lights (black Kd, bright Ke) keep their color
    return {std::fmaxf(m.kd.x, std::fminf(m.ke.x, 1.0f)),
            std::fmaxf(m.kd.y, std::fminf(m.ke.y, 1.0f)),
            std::fmaxf(m.kd.z, std::fminf(m.ke.z, 1.0f))};
}

V3 xform_point(const float* T, const V3& p)
{
    return {T[0] * p.x + T[1] * p.y + T[2] * p.z + T[9],
            T[3] * p.x + T[4] * p.y + T[5] * p.z + T[10],
            T[6] * p.x + T[7] * p.y + T[8] * p.z + T[11]};
}

V3 xform_dir(const float* T, const V3& p)
{
    return {T[0] * p.x + T[1] * p.y + T[2] * p.z,
            T[3] * p.x + T[4] * p.y + T[5] * p.z,
            T[6] * p.x + T[7] * p.y + T[8] * p.z};
}

V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

V3 cross(const V3& a, const V3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

V3 normalized(const V3& a)
{
    float n = std::sqrt(dot(a, a));
    if (n < 1e-20f) return {0, 0, 1};
    return {a.x / n, a.y / n, a.z / n};
}

// MTL subset: newmtl / Kd / Ke. Unreadable file -> empty map (caller grays out).
std::map<std::string, Mat> parse_mtl(const std::string& path)
{
    std::map<std::string, Mat> mats;
    std::ifstream in(path);
    std::string line, cur;
    while (std::getline(in, line))
    {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "newmtl")
        {
            ss >> cur;
            mats[cur] = Mat{};
            mats[cur].kd = {0.0f, 0.0f, 0.0f};  // real material: Kd replaces gray
        }
        else if (tag == "Kd" && !cur.empty())
        {
            ss >> mats[cur].kd.x >> mats[cur].kd.y >> mats[cur].kd.z;
        }
        else if (tag == "Ke" && !cur.empty())
        {
            ss >> mats[cur].ke.x >> mats[cur].ke.y >> mats[cur].ke.z;
        }
    }
    return mats;
}

// "v", "v/vt", "v//vn", "v/vt/vn" -> (vertex idx, normal idx); OBJ indices are
// 1-based, negative = relative to end. Returns 0-based, normal -1 if absent.
void parse_corner(const std::string& tok, std::size_t nv, std::size_t nn,
                  long& vi, long& ni)
{
    long v = std::strtol(tok.c_str(), nullptr, 10);
    long n = 0;
    bool has_n = false;
    std::size_t s1 = tok.find('/');
    if (s1 != std::string::npos)
    {
        std::size_t s2 = tok.find('/', s1 + 1);
        if (s2 != std::string::npos && s2 + 1 < tok.size())
        {
            n = std::strtol(tok.c_str() + s2 + 1, nullptr, 10);
            has_n = true;
        }
    }
    vi = v > 0 ? v - 1 : static_cast<long>(nv) + v;
    ni = has_n ? (n > 0 ? n - 1 : static_cast<long>(nn) + n) : -1;
}
}  // namespace

RobotMesh load_obj_mesh(const std::string& obj_path, const float* T)
{
    std::ifstream in(obj_path);
    if (!in) throw std::runtime_error("cannot open OBJ: " + obj_path);

    // MTL lives next to the OBJ (Blender convention).
    std::string dir;
    std::size_t slash = obj_path.find_last_of('/');
    if (slash != std::string::npos) dir = obj_path.substr(0, slash + 1);

    std::vector<V3> pos, nrm;                    // already transformed to rig frame
    std::map<std::string, Mat> mats;
    V3 cur_col = {0.7f, 0.7f, 0.7f};             // gray fallback (missing MTL/material)
    RobotMesh mesh;

    std::string line;
    while (std::getline(in, line))
    {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v")
        {
            V3 p;
            ss >> p.x >> p.y >> p.z;
            pos.push_back(xform_point(T, p));
        }
        else if (tag == "vn")
        {
            V3 n;
            ss >> n.x >> n.y >> n.z;
            nrm.push_back(xform_dir(T, n));
        }
        else if (tag == "mtllib")
        {
            std::string f;
            ss >> f;
            mats = parse_mtl(dir + f);
        }
        else if (tag == "usemtl")
        {
            std::string m;
            ss >> m;
            auto it = mats.find(m);
            cur_col = it != mats.end() ? mat_color(it->second) : V3{0.7f, 0.7f, 0.7f};
        }
        else if (tag == "f")
        {
            // ponytail: 128-corner cap — covers every face in M02P (max 100-gon);
            // longer faces lose their tail corners, visually nil on a flat disc.
            long vi[128], ni[128];
            int nc = 0;
            std::string tok;
            while (ss >> tok && nc < 128)
            {
                parse_corner(tok, pos.size(), nrm.size(), vi[nc], ni[nc]);
                ++nc;
            }
            for (int k = 1; k + 1 < nc; ++k)  // fan-triangulate (0, k, k+1)
            {
                int idx[3] = {0, k, k + 1};
                V3 p[3], n_avg = {0, 0, 0};
                bool all_n = true;
                bool bad = false;
                for (int c = 0; c < 3; ++c)
                {
                    long v = vi[idx[c]];
                    if (v < 0 || v >= static_cast<long>(pos.size()))
                    {
                        bad = true;
                        break;
                    }
                    p[c] = pos[static_cast<std::size_t>(v)];
                    long nn = ni[idx[c]];
                    if (nn >= 0 && nn < static_cast<long>(nrm.size()))
                    {
                        n_avg.x += nrm[static_cast<std::size_t>(nn)].x;
                        n_avg.y += nrm[static_cast<std::size_t>(nn)].y;
                        n_avg.z += nrm[static_cast<std::size_t>(nn)].z;
                    }
                    else
                    {
                        all_n = false;
                    }
                }
                if (bad) continue;  // malformed index -> skip triangle, not the file
                V3 n = all_n ? normalized(n_avg)
                             : normalized(cross(sub(p[1], p[0]), sub(p[2], p[0])));
                // winding-robust lambert: |dot| avoids black facets on
                // inconsistently-wound OBJ shells
                float shade = 0.35f + 0.65f * std::fabs(dot(n, kLight));
                for (int c = 0; c < 3; ++c)
                {
                    mesh.verts.push_back(p[c].x);
                    mesh.verts.push_back(p[c].y);
                    mesh.verts.push_back(p[c].z);
                }
                mesh.cols.push_back(cur_col.x * shade);
                mesh.cols.push_back(cur_col.y * shade);
                mesh.cols.push_back(cur_col.z * shade);
            }
        }
    }
    mesh.n_tris = mesh.cols.size() / 3;
    if (mesh.n_tris == 0) throw std::runtime_error("no faces in OBJ: " + obj_path);
    return mesh;
}
}  // namespace micropilot::rendering
