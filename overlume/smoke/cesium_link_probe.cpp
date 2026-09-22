// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/LocalHorizontalCoordinateSystem.h>
#include <CesiumGltfReader/GltfReader.h>
#include <cmath>
#include <cstdio>
int main() {
    const auto& wgs84 = CesiumGeospatial::Ellipsoid::WGS84;
    CesiumGeospatial::LocalHorizontalCoordinateSystem enu(
        CesiumGeospatial::Cartographic::fromDegrees(55.3910, 25.0803, 0.0));
    auto ecef = enu.localPositionToEcef(glm::dvec3(0.0, 0.0, 0.0));
    double mag = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    if (!(mag > 6.35e6 && mag < 6.40e6)) {
        std::puts("FAIL ecef");
        return 1;
    }
    CesiumGltfReader::GltfReader reader;
    (void)reader;
    (void)wgs84;
    std::puts("cesium_link_probe OK");
    return 0;
}
