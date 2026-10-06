# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_install_mesa.ps1 -- self-test of install_mesa.ps1's exit-code contract: a corrupt cached archive on
# x64 must exit 1 (the callers fail the job), arm64 must exit 2 (the documented no-GPU path).
$ErrorActionPreference = 'Stop'
$m = Join-Path $PSScriptRoot 'install_mesa.ps1'
$dir = Join-Path ([IO.Path]::GetTempPath()) 'mesa-26.2.4'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
$ar = Join-Path $dir 'mesa3d-26.2.4-release-msvc.7z'
$dest = Join-Path ([IO.Path]::GetTempPath()) 'mesa-selftest-dest'
Set-Content -Path $ar -Value 'junk'
& $m -Arch x64 -Dest $dest | Out-Null
if ($LASTEXITCODE -ne 1) { Write-Output "FAIL: install_mesa x64 with a corrupt archive exited $LASTEXITCODE, want 1"; exit 1 }
& $m -Arch arm64 -Dest $dest | Out-Null
if ($LASTEXITCODE -ne 2) { Write-Output "FAIL: install_mesa arm64 exited $LASTEXITCODE, want 2"; exit 1 }
Write-Output 'PASS: install_mesa exit-code contract'
exit 0
