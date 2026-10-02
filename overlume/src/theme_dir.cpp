// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "theme_dir.hpp"

#include <filesystem>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace overlume::detail {

std::string resolve_default_theme_dir(const std::string& module_path,
                                      const char* compiled_default) {
    namespace fs = std::filesystem;
    if (!module_path.empty()) {
        std::error_code ec;
        const fs::path dir =
            (fs::path(module_path).parent_path() / ".." / "share" / "overlume" / "themes")
                .lexically_normal();
        if (fs::is_directory(dir, ec)) {
            for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
                if (it->path().extension() == ".yaml") return dir.string();
            }
        }
    }
    return compiled_default;
}

std::string current_module_path() {
#ifdef _WIN32
    HMODULE mod = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            reinterpret_cast<LPCWSTR>(&current_module_path), &mod))
        return "";
    wchar_t buf[MAX_PATH * 4];
    const DWORD n = GetModuleFileNameW(mod, buf, static_cast<DWORD>(sizeof(buf) / sizeof(buf[0])));
    if (n == 0 || n >= sizeof(buf) / sizeof(buf[0])) return "";
    return std::filesystem::path(std::wstring(buf, n)).string();
#else
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&current_module_path), &info) == 0 || !info.dli_fname)
        return "";
    std::error_code ec;
    const auto abs = std::filesystem::absolute(info.dli_fname, ec);
    return ec ? std::string(info.dli_fname) : abs.string();
#endif
}

}  // namespace overlume::detail
