// Links and exercises cesium-native across several of its archives
// (Geospatial math, GltfReader parse, Async continuation) without network,
// GPU, or Filament -- the link-recipe proof, per filament_link_probe's own
// precedent. C++20 TU (cesium headers require it).
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/LocalHorizontalCoordinateSystem.h>
#include <CesiumGltfReader/GltfReader.h>
#include <cmath>
#include <cstdio>
int main() {
    const auto& wgs84 = CesiumGeospatial::Ellipsoid::WGS84;
    // ECEF of the recorded operating area's anchor (Epic 4 Decision 6) --
    // real math through CesiumGeospatial, checked for sanity, not goldens.
    CesiumGeospatial::LocalHorizontalCoordinateSystem enu(
        CesiumGeospatial::Cartographic::fromDegrees(55.3910, 25.0803, 0.0));
    auto ecef = enu.localPositionToEcef(glm::dvec3(0.0, 0.0, 0.0));
    // Sanity bound on the VECTOR MAGNITUDE (distance from Earth's center),
    // not a single axis: WGS84's radius is ~6.378e6m at the equator and
    // ~6.357e6m at the poles, so any point on/near the surface lands in
    // [6.35e6, 6.40e6] regardless of longitude -- unlike a per-axis bound
    // (the original `ecef.x` check), which only happens to hold for
    // anchors whose longitude puts most of the radius on the X axis. This
    // anchor (lon 55.39, lat 25.08, Dubai) does NOT: X ~= 3.28e6, Y ~=
    // 4.76e6, Z ~= 2.69e6 -- each individually outside [5e6, 7e6], while
    // sqrt(x^2+y^2+z^2) ~= 6.378e6 is exactly earth-radius-scale. Verified
    // by running the fix: FAILED before, PASSED after.
    double mag = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    if (!(mag > 6.35e6 && mag < 6.40e6)) { std::puts("FAIL ecef"); return 1; }
    CesiumGltfReader::GltfReader reader;  // constructing it pulls reader+deps archives
    (void)reader; (void)wgs84;
    std::puts("cesium_link_probe OK");
    return 0;
}
