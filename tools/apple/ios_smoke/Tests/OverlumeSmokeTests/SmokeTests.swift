// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
import XCTest
import OverlumeShim

final class SmokeTests: XCTestCase {
    func testFrameworkLinksAndRejectsZeroSize() {
        XCTAssertEqual(overlume_shim_zero_size_is_null(), 1)
    }

    // 0 (no Metal device on this runner) is a legitimate answer; the log line says which happened.
    // Hosted macOS runners back the simulator GPU with a paravirtual device Filament aborts on (CI
    // 37337587866); build_xcframework.sh smoke sets this there and warns. A real Mac renders.
    func testRenderOrNoGpu() throws {
        if ProcessInfo.processInfo.environment["OVERLUME_SMOKE_SKIP_RENDER"] == "1" {
            throw XCTSkip("simulator GPU is paravirtual on this runner; Metal frame not rendered")
        }
        let r = overlume_shim_render()
        print("OVERLUME_SMOKE render=\(r) (0 = no Metal device, 2 = frame + themes)")
        XCTAssertTrue(r == 0 || r == 2, "render result \(r)")
    }
}
