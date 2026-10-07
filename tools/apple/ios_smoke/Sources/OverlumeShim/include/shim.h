// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// create_renderer with a 0x0 config: must return nullptr (1), proving the framework links and runs.
int overlume_shim_zero_size_is_null(void);
// 0 = no Metal device (create_renderer returned nullptr), 2 = frame rendered with the framework's
// own Resources/themes found, 1 = rendered but the themes were not found, -1 = frame failed.
int overlume_shim_render(void);
#ifdef __cplusplus
}
#endif
