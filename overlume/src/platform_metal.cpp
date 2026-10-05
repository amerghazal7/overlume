// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "platform.hpp"

#include <objc/message.h>
#include <objc/runtime.h>

#include <cstring>

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
        // The base class declares newArgumentEncoderWithLayout: (CI 37150239822: the probe passed and
        // Filament still threw), so identify the paravirtual device by its name instead.
        auto msg = reinterpret_cast<id (*)(id, SEL)>(objc_msgSend);
        auto utf8 = reinterpret_cast<const char* (*)(id, SEL)>(objc_msgSend);
        const id name = msg(reinterpret_cast<id>(device), sel_registerName("name"));
        const char* n = name != nullptr ? utf8(name, sel_registerName("UTF8String")) : nullptr;
        usable = n == nullptr || std::strstr(n, "Paravirtual") == nullptr;
        objc_release(device);
    }
    return HeadlessPlatform{nullptr, filament::Engine::Backend::METAL, usable};
}

void destroy_headless_platform(HeadlessPlatform&) {}

}  // namespace overlume::detail
