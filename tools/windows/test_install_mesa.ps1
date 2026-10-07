# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_install_mesa.ps1 -- self-test of (1) install_mesa.ps1's exit-code contract (a corrupt cached archive
# must exit 1 on x64 and arm64) and (2) build_windows.ps1 test's fail-closed decision table, driven with a
# stub install_mesa.ps1 and a stub ctest function: install exit 1 fails, no GL context with Mesa present
# fails, and only a no-Mesa (exit 2) run may take the ::warning:: path. (smoke_windows.ps1's twin guards are
# not covered here: stubbing the vcpkg and consumer builds that run first is too heavy.)
$ErrorActionPreference = 'Stop'
$m = Join-Path $PSScriptRoot 'install_mesa.ps1'
$dest = Join-Path ([IO.Path]::GetTempPath()) 'mesa-selftest-dest'
$dir = Join-Path ([IO.Path]::GetTempPath()) 'mesa-26.2.4'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
foreach ($p in @(@('x64', 'mesa3d-26.2.4-release-msvc.7z'), @('arm64', 'mesa-llvmpipe-arm64-26.2.4.7z'))) {
    Set-Content -Path (Join-Path $dir $p[1]) -Value 'junk'
    & $m -Arch $p[0] -Dest $dest | Out-Null
    if ($LASTEXITCODE -ne 1) { Write-Output "FAIL: install_mesa $($p[0]) with a corrupt archive exited $LASTEXITCODE, want 1"; exit 1 }
}

# Caller decision table.
$t = Join-Path ([IO.Path]::GetTempPath()) 'mesa-selftest-caller'
Remove-Item -Recurse -Force $t -ErrorAction SilentlyContinue
$tw = Join-Path $t 'tools\windows'
New-Item -ItemType Directory -Force -Path $tw | Out-Null
Copy-Item (Join-Path $PSScriptRoot 'build_windows.ps1') $tw
Set-Content -Path (Join-Path $tw 'install_mesa.ps1') -Value 'exit [int]$env:STUB_RC'
$copy = Join-Path $tw 'build_windows.ps1'
function ctest { $global:LASTEXITCODE = 0; if ($args -contains '-R') { 'No GPU/EGL device available' } }
function Run($rc, $arch) {
    $env:STUB_RC = "$rc"
    $o = & $copy test -Arch $arch 2>&1 | Out-String
    [pscustomobject]@{ Rc = $LASTEXITCODE; Out = $o }
}
$r = Run 1 'x64'
if ($r.Rc -ne 1 -or $r.Out -notmatch 'Mesa llvmpipe install') { Write-Output "FAIL: install exit 1 must fail the test step (rc=$($r.Rc))"; exit 1 }
$r = Run 0 'x64'
if ($r.Rc -ne 1 -or $r.Out -notmatch 'despite Mesa') { Write-Output "FAIL: no GL context with Mesa present must fail (rc=$($r.Rc))"; exit 1 }
$r = Run 2 'arm64'
if ($r.Rc -ne 0 -or $r.Out -notmatch '::warning::') { Write-Output "FAIL: no-Mesa run must warn and pass (rc=$($r.Rc))"; exit 1 }
Write-Output 'PASS: install_mesa exit-code contract and build_windows fail-closed table'
exit 0
