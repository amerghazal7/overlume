// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "platform.hpp"

// Filament's default PlatformMetal drives a headless readable swap chain
// (createSwapChain(w, h, CONFIG_READABLE)); no platform object of our own is needed. Only a
// device probe is: without an MTLDevice (virtualised CI runners) Engine::Builder::build() would
// abort instead of returning nullptr.
extern "C" {
void* MTLCreateSystemDefaultDevice(void);  // Metal.framework; returns a +1 object
void objc_release(void*);                  // libobjc
}

namespace overlume::detail {

HeadlessPlatform make_headless_platform() {
    void* device = MTLCreateSystemDefaultDevice();
    HeadlessPlatform p{nullptr, filament::Engine::Backend::METAL, device != nullptr};
    if (device != nullptr) objc_release(device);
    return p;
}

void destroy_headless_platform(HeadlessPlatform&) {}

}  // namespace overlume::detail
