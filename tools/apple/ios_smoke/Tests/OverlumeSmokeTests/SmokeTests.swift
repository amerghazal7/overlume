// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
import XCTest
import OverlumeShim

final class SmokeTests: XCTestCase {
    func testFrameworkLinksAndRejectsZeroSize() {
        XCTAssertEqual(overlume_shim_zero_size_is_null(), 1)
    }

    // 0 (no Metal device on this runner) is a legitimate answer; the log line says which happened.
    func testRenderOrNoGpu() {
        let r = overlume_shim_render()
        print("OVERLUME_SMOKE render=\(r) (0 = no Metal device, 2 = frame + themes)")
        XCTAssertTrue(r == 0 || r == 2, "render result \(r)")
    }
}
