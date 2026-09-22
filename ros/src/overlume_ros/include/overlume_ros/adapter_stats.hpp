// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

namespace overlume::ros {

struct AdapterStats {
    double last_msg_sec{0.0};
    uint64_t msgs{0};
    uint64_t dropped_malformed{0};
    uint64_t dropped_stale{0};
    uint64_t dropped_no_tf{0};
    uint64_t dropped_by_rule{0};
};

}  // namespace overlume::ros
