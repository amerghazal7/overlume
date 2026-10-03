// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "platform.hpp"

#include <objc/runtime.h>

// Filament's default PlatformMetal drives a headless readable swap chain
// (createSwapChain(w, h, CONFIG_READABLE)); no platform object of our own is needed. Only a
// device probe is: Engine::Builder::build() aborts (an Objective-C exception) instead of
// returning nullptr on a device Filament cannot use. The GitHub macOS runners' paravirtual GPU
// ("AppleParavirtDevice") is one: it exists but has no argument-encoder support, which Filament's
// Metal driver needs.
extern "C" {
void* MTLCreateSystemDefaultDevice(void);  // Metal.framework; returns a +1 object
void objc_release(void*);                  // libobjc
}

namespace overlume::detail {

HeadlessPlatform make_headless_platform() {
    void* device = MTLCreateSystemDefaultDevice();
    bool usable = false;
    if (device != nullptr) {
        usable = class_respondsToSelector(object_getClass(reinterpret_cast<id>(device)),
                                          sel_registerName("newArgumentEncoderWithLayout:"));
        objc_release(device);
    }
    return HeadlessPlatform{nullptr, filament::Engine::Backend::METAL, usable};
}

void destroy_headless_platform(HeadlessPlatform&) {}

}  // namespace overlume::detail
