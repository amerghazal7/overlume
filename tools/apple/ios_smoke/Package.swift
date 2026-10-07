// swift-tools-version:5.9
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
//
// Consumer of Overlume.xcframework (copied next to this file by build_xcframework.sh smoke): a
// C++ shim over the public API plus an XCTest that runs in the iOS simulator.
import PackageDescription

let package = Package(
    name: "OverlumeSmoke",
    platforms: [.iOS(.v15)],
    products: [.library(name: "OverlumeShim", targets: ["OverlumeShim"])],
    targets: [
        .binaryTarget(name: "Overlume", path: "Overlume.xcframework"),
        .target(name: "OverlumeShim", dependencies: ["Overlume"], path: "Sources/OverlumeShim",
                cxxSettings: [.unsafeFlags(["-std=gnu++17"])]),
        .testTarget(name: "OverlumeSmokeTests", dependencies: ["OverlumeShim"],
                    path: "Tests/OverlumeSmokeTests"),
    ]
)
