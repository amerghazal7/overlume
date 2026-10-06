// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#include <overlume/api.h>
#include <overlume/version.h>

#include <cstdio>

int main() {
    overlume::RenderConfig config{64, 64, 1, nullptr, "dark_adas"};
    overlume::VisualRenderer* r = overlume::create_renderer(config);  // nullptr without a GPU: still a link+load proof
    std::printf("overlume %s renderer=%s\n", overlume::kVersionString, r ? "yes" : "no");
    if (r) overlume::destroy_renderer(r);
    return 0;
}
