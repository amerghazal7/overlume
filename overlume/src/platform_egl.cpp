// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "platform.hpp"

#include <backend/platforms/OpenGLPlatform.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <cstdio>

#if defined(__ANDROID__)
#include <GLES3/gl3.h>
// libbackend.a: resolves the GLES extension entry points (glInsertEventMarkerEXT, ...) through
// eglGetProcAddress; Filament's own PlatformEGL calls it, this platform must too.
namespace glext {
void importGLESExtensionsEntryPoints();
}
#else
namespace bluegl {
int bind();
void unbind();
}

extern "C" const unsigned char* bluegl_glGetString(unsigned int name);
#endif
constexpr unsigned int kGlVendor = 0x1F00;
constexpr unsigned int kGlRenderer = 0x1F01;
constexpr unsigned int kGlVersion = 0x1F02;

namespace overlume::detail {

namespace {

class HeadlessEglPlatform : public filament::backend::OpenGLPlatform {
public:
    struct EglSwapChain : public filament::backend::Platform::SwapChain {
        EGLSurface surface = EGL_NO_SURFACE;
    };

    int getOSVersion() const noexcept override { return 0; }

    filament::backend::Driver* createDriver(void*,
                                            const DriverConfig& driverConfig) noexcept override {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display_ == EGL_NO_DISPLAY) return nullptr;
        if (eglInitialize(display_, nullptr, nullptr) != EGL_TRUE) return nullptr;
#if defined(__ANDROID__)
        glext::importGLESExtensionsEntryPoints();
        if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE) return nullptr;
#else
        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) return nullptr;
#endif

        const EGLint configAttribs[] = {
            EGL_SURFACE_TYPE,
            EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE,
#if defined(__ANDROID__)
            EGL_OPENGL_ES3_BIT,
#else
            EGL_OPENGL_BIT,
#endif
            EGL_RED_SIZE,
            8,
            EGL_GREEN_SIZE,
            8,
            EGL_BLUE_SIZE,
            8,
            EGL_ALPHA_SIZE,
            8,
            EGL_DEPTH_SIZE,
            24,
            EGL_STENCIL_SIZE,
            8,
            EGL_NONE,
        };
        EGLint numConfigs = 0;
        if (eglChooseConfig(display_, configAttribs, &config_, 1, &numConfigs) != EGL_TRUE ||
            numConfigs == 0) {
            return nullptr;
        }

#if defined(__ANDROID__)
        const EGLint ctxAttribs[] = {
            EGL_CONTEXT_CLIENT_VERSION,
            3,
            EGL_NONE,
        };
#else
        const EGLint ctxAttribs[] = {
            EGL_CONTEXT_MAJOR_VERSION,
            4,
            EGL_CONTEXT_MINOR_VERSION,
            5,
            EGL_CONTEXT_OPENGL_PROFILE_MASK,
            EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        };
#endif
        context_ = eglCreateContext(display_, config_, EGL_NO_CONTEXT, ctxAttribs);
        if (context_ == EGL_NO_CONTEXT) return nullptr;

        const EGLint bootstrapAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        bootstrapSurface_ = eglCreatePbufferSurface(display_, config_, bootstrapAttribs);
        if (bootstrapSurface_ == EGL_NO_SURFACE) return nullptr;
        if (eglMakeCurrent(display_, bootstrapSurface_, bootstrapSurface_, context_) != EGL_TRUE) {
            return nullptr;
        }
#if !defined(__ANDROID__)
        if (bluegl::bind() != 0) return nullptr;
        blueglBound_ = true;
#endif

        const auto gl_str = [](unsigned int n) {
#if defined(__ANDROID__)
            const unsigned char* s = glGetString(n);
#else
            const unsigned char* s = bluegl_glGetString(n);
#endif
            return s != nullptr ? reinterpret_cast<const char*>(s) : "(null)";
        };
        std::fprintf(stderr, "[overlume] GL_VENDOR: %s\n", gl_str(kGlVendor));
        std::fprintf(stderr, "[overlume] GL_RENDERER: %s\n", gl_str(kGlRenderer));
        std::fprintf(stderr, "[overlume] GL_VERSION: %s\n", gl_str(kGlVersion));

        return createDefaultDriver(this, nullptr, driverConfig);
    }

    filament::backend::Platform::SwapChain* createSwapChain(void*, uint64_t) noexcept override {
        return nullptr;
    }

    filament::backend::Platform::SwapChain* createSwapChain(uint32_t width, uint32_t height,
                                                            uint64_t) noexcept override {
        const EGLint pbufferAttribs[] = {
            EGL_WIDTH, static_cast<EGLint>(width), EGL_HEIGHT, static_cast<EGLint>(height),
            EGL_NONE,
        };
        EGLSurface surface = eglCreatePbufferSurface(display_, config_, pbufferAttribs);
        if (surface == EGL_NO_SURFACE) return nullptr;
        auto* swapChain = new EglSwapChain();
        swapChain->surface = surface;
        return swapChain;
    }

    void destroySwapChain(filament::backend::Platform::SwapChain* swapChain) noexcept override {
        auto* sc = static_cast<EglSwapChain*>(swapChain);
        if (sc->surface != EGL_NO_SURFACE) eglDestroySurface(display_, sc->surface);
        delete sc;
    }

    bool makeCurrent(ContextType, filament::backend::Platform::SwapChain* drawSwapChain,
                     filament::backend::Platform::SwapChain* readSwapChain) noexcept override {
        auto* draw = static_cast<EglSwapChain*>(drawSwapChain);
        auto* read = static_cast<EglSwapChain*>(readSwapChain);
        return eglMakeCurrent(display_, draw->surface, read->surface, context_) == EGL_TRUE;
    }

    void commit(filament::backend::Platform::SwapChain*) noexcept override {}

    void terminate() noexcept override {
        if (display_ == EGL_NO_DISPLAY) return;
#if !defined(__ANDROID__)
        if (blueglBound_) {
            bluegl::unbind();
            blueglBound_ = false;
        }
#endif
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (bootstrapSurface_ != EGL_NO_SURFACE) eglDestroySurface(display_, bootstrapSurface_);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
        eglTerminate(display_);
        display_ = EGL_NO_DISPLAY;
    }

private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLConfig config_ = nullptr;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface bootstrapSurface_ = EGL_NO_SURFACE;
    bool blueglBound_ = false;
};

}  // namespace

HeadlessPlatform make_headless_platform() {
    return {new HeadlessEglPlatform(), filament::Engine::Backend::OPENGL};
}

void destroy_headless_platform(HeadlessPlatform& p) {
    delete p.platform;
    p.platform = nullptr;
}

}  // namespace overlume::detail
