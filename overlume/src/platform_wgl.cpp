// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "platform.hpp"

#include <windows.h>

#include <cstdio>

// Filament's default PlatformWGL drives a headless swap chain (a hidden WS_POPUP window), so no
// platform object of our own is needed. Only a probe is: its createDriver() calls the
// wglCreateContextAttribsARB pointer unchecked, so a machine whose OpenGL is the 1.1 GDI generic
// implementation (no GPU driver, no Mesa) would crash inside Engine::Builder::build() instead of
// returning nullptr. create_renderer() returns nullptr when no OpenGL 4.1 core context can be made.
namespace overlume::detail {

namespace {

bool opengl_41_available() {
    HWND wnd = CreateWindowA("STATIC", "overlume-probe", 0, 0, 0, 1, 1, nullptr, nullptr, nullptr,
                             nullptr);
    if (wnd == nullptr) return false;
    HDC dc = GetDC(wnd);
    bool ok = false;
    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;
    const int format = dc != nullptr ? ChoosePixelFormat(dc, &pfd) : 0;
    if (format != 0 && SetPixelFormat(dc, format, &pfd)) {
        HGLRC legacy = wglCreateContext(dc);
        if (legacy != nullptr && wglMakeCurrent(dc, legacy)) {
            using CreateContextAttribs = HGLRC(WINAPI*)(HDC, HGLRC, const int*);
            auto create = reinterpret_cast<CreateContextAttribs>(
                reinterpret_cast<void*>(wglGetProcAddress("wglCreateContextAttribsARB")));
            if (create != nullptr) {
                constexpr int kContextMajorVersion = 0x2091;  // WGL_CONTEXT_MAJOR_VERSION_ARB
                constexpr int kContextMinorVersion = 0x2092;  // WGL_CONTEXT_MINOR_VERSION_ARB
                const int attribs[] = {kContextMajorVersion, 4, kContextMinorVersion, 1, 0};
                HGLRC modern = create(dc, nullptr, attribs);
                if (modern != nullptr) {
                    ok = true;
                    wglDeleteContext(modern);
                }
            }
            wglMakeCurrent(nullptr, nullptr);
        }
        if (legacy != nullptr) wglDeleteContext(legacy);
    }
    if (dc != nullptr) ReleaseDC(wnd, dc);
    DestroyWindow(wnd);
    return ok;
}

}  // namespace

HeadlessPlatform make_headless_platform() {
    const bool usable = opengl_41_available();
    if (!usable) {
        std::fprintf(stderr, "[overlume] no OpenGL 4.1 context available (no GPU driver?)\n");
    }
    return HeadlessPlatform{nullptr, filament::Engine::Backend::OPENGL, usable};
}

void destroy_headless_platform(HeadlessPlatform&) {}

}  // namespace overlume::detail
