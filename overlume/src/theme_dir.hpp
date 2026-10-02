// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>

namespace overlume::detail {

// `<dirname(module_path)>/../share/overlume/themes` when that is a directory
// holding at least one *.yaml (an installed package), else `compiled_default`.
std::string resolve_default_theme_dir(const std::string& module_path,
                                      const char* compiled_default);

// Absolute path of the shared object / DLL containing this code, or "" when
// unknown (static link into an executable still yields the executable path).
std::string current_module_path();

}  // namespace overlume::detail
