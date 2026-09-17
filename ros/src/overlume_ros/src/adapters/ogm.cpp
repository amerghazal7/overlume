#include "overlume_ros/adapters/ogm.hpp"

#include <cmath>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume_node
{
namespace
{

uint8_t KindFromRole(const std::string& role)
{
    // profile.cpp's ValidateRow already restricts adapter: ogm rows to this
    // closed two-value set -- anything else fails profile load before an
    // adapter is ever constructed, same "row.role, never the topic name"
    // rule PathAdapter's RoleFromString documents.
    if (role == "gradient_ogm") return 1;
    return 0;  // "dynamic_ogm"
}

bool HasNan(const tf2::Vector3& v) { return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z()); }

// The ONE int8->uint8 conversion shared by the full-grid and patch paths.
// Never a memcpy/static_cast -- see ogm.hpp for why. Sets malformed=true
// (never cleared) so callers can ++dropped_malformed once per message
// regardless of how many cells were out of range.
uint8_t ConvertCell(int8_t v, bool& malformed)
{
    if (v == -1) return kUnknownCell;
    if (v >= 0 && v <= 100) return static_cast<uint8_t>(v);
    malformed = true;
    return kUnknownCell;
}

}  // namespace

OgmAdapter::OgmAdapter(const ProfileRow& row,
                       const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), kind_(KindFromRole(row.role))
{
}

void OgmAdapter::ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec)
{
    ++stats_.msgs;

    // Malformed guards (spec §9): a zero-sized grid, or a data length that
    // doesn't match width*height, has nothing coherent to store -- drop the
    // WHOLE message, previously-stored grid (if any) keeps rendering.
    if (msg.info.width == 0 || msg.info.height == 0)
    {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.info.width) * msg.info.height)
    {
        ++stats_.dropped_malformed;
        return;
    }

    // ONE lookup for the whole message, transforms the grid's origin (cell
    // (0,0)'s pose) into the map frame. GroundGridLayer::origin is
    // position-only; origin.orientation is NOT applied (same convention
    // PathRibbon/MapElement accept). FIXTURE GAP 3: no OGM publisher exists
    // in the recorded stack to confirm whether a real one ships non-identity
    // origin orientation.
    tf2::Transform xform;
    if (!tf_.lookup(msg.header, xform))
    {
        ++stats_.dropped_no_tf;
        return;  // whole message dropped; previously-stored grid stays
    }
    const auto& p = msg.info.origin.position;
    const tf2::Vector3 originTf = xform * tf2::Vector3(p.x, p.y, p.z);
    if (HasNan(originTf))
    {
        ++stats_.dropped_malformed;  // NaN /tf entry -- drop the whole message
        return;
    }

    // Conversion, not memcpy (see ogm.hpp/this file's ConvertCell): one
    // dropped_malformed per MESSAGE, however many individual cells were out
    // of range.
    std::vector<uint8_t> next(msg.data.size());
    bool malformed = false;
    for (size_t i = 0; i < msg.data.size(); ++i)
    {
        next[i] = ConvertCell(msg.data[i], malformed);
    }
    if (malformed) ++stats_.dropped_malformed;

    // REPLACES the stored grid wholesale -- dims, origin, resolution, every
    // cell (a fresh full grid is the only way real OGM dims ever change;
    // see ingest_update()'s "never resizes" contract below).
    cells_ = std::move(next);
    // flatten_z: 2D HD-map plane -- see frame_transform.hpp.
    origin_ = {originTf.x(), originTf.y(), tf_.flatten_z() ? 0.0 : originTf.z()};
    resolution_m_ = msg.info.resolution;
    width_cells_ = msg.info.width;
    height_cells_ = msg.info.height;
    has_grid_ = true;
    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void OgmAdapter::ingest_update(const map_msgs::msg::OccupancyGridUpdate& msg, double sim_time_sec)
{
    ++stats_.msgs;

    // No base grid yet -- nothing to patch. Dropped + counted, no allocation,
    // no crash. Permanent if `ogm` ever stops being one row with two
    // subscriptions (see OneRowYieldsTwoSubscriptionsAndOneAdapter).
    if (!has_grid_)
    {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.width == 0 || msg.height == 0)
    {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.width) * msg.height)
    {
        ++stats_.dropped_malformed;
        return;
    }
    // x/y are int32 on the wire (map_msgs/OccupancyGridUpdate.msg) -- a
    // negative offset is already out of bounds against a uint32-dimensioned
    // base grid.
    if (msg.x < 0 || msg.y < 0)
    {
        ++stats_.dropped_malformed;
        return;
    }
    const uint32_t x0 = static_cast<uint32_t>(msg.x);
    const uint32_t y0 = static_cast<uint32_t>(msg.y);
    // Overflow-safe bound check (width_cells_ - x0 would wrap if x0 >
    // width_cells_, so compare via addition-free subtraction only after
    // confirming x0/y0 themselves are in range).
    if (x0 > width_cells_ || y0 > height_cells_)
    {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.width > width_cells_ - x0 || msg.height > height_cells_ - y0)
    {
        ++stats_.dropped_malformed;
        return;
    }

    // In-place patch: exactly the (x0,y0,width,height) sub-rectangle is
    // rewritten; every cell outside it is untouched (PartialUpdatePatches
    // InPlaceWithoutResizing) -- cells_ itself is never reallocated here.
    bool malformed = false;
    for (uint32_t row = 0; row < msg.height; ++row)
    {
        const uint32_t destRowStart = (y0 + row) * width_cells_ + x0;
        const uint32_t srcRowStart = row * msg.width;
        for (uint32_t col = 0; col < msg.width; ++col)
        {
            cells_[destRowStart + col] = ConvertCell(msg.data[srcRowStart + col], malformed);
        }
    }
    if (malformed) ++stats_.dropped_malformed;

    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void OgmAdapter::fill(overlume::ros::SceneAssembly& out) const
{
    if (!has_grid_) return;  // never received a valid full grid yet

    overlume::GroundGridLayer g{};
    g.kind = kind_;
    g.origin = origin_;
    g.resolution_m = resolution_m_;
    g.width_cells = width_cells_;
    g.height_cells = height_cells_;
    g.cells = cells_.data();
    g.last_update_sec = last_update_sec_;
    out.grids.push_back(g);
}

}  // namespace overlume_node
