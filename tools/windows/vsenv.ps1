# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# vsenv.ps1 -Arch x64|arm64 -- exports the Visual Studio (MSVC v143) build environment for the target
# architecture into $GITHUB_ENV (what vcvarsall.bat sets), puts a clang-cl on PATH (the VS-bundled one
# when none is installed), and prints the tool versions. The build uses clang-cl: Filament's headers
# and its prebuilt Windows libraries are clang-cl products; the ABI and CRT (/MD) are MSVC's.
param([Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch)
$ErrorActionPreference = 'Stop'

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -property installationPath
if (-not $vs) { Write-Output 'FAIL: no Visual Studio installation found'; exit 1 }
$bat = Join-Path $vs 'VC\Auxiliary\Build\vcvarsall.bat'
$hostArch = if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64') { 'arm64' } else { 'x64' }
$vcArg = if ($Arch -eq $hostArch) { $Arch } else { "${hostArch}_$Arch" }
Write-Output "Visual Studio: $vs (vcvarsall $vcArg)"

$lines = cmd /c "`"$bat`" $vcArg >nul && set"
if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL: vcvarsall.bat failed'; exit 1 }
foreach ($l in $lines) {
    if ($l -match '^([^=]+)=(.*)$') {
        Set-Item -Path "env:$($Matches[1])" -Value $Matches[2]
        if ($env:GITHUB_ENV) { "$($Matches[1])=$($Matches[2])" | Out-File -Append -Encoding utf8 $env:GITHUB_ENV }
    }
}

if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) {
    $llvm = Get-ChildItem -Path (Join-Path $vs 'VC\Tools\Llvm') -Directory -ErrorAction SilentlyContinue |
        Where-Object { Test-Path (Join-Path $_.FullName 'bin\clang-cl.exe') } | Select-Object -First 1
    if ($llvm) {
        $bin = Join-Path $llvm.FullName 'bin'
        $env:PATH = "$bin;$env:PATH"
        if ($env:GITHUB_PATH) { $bin | Out-File -Append -Encoding utf8 $env:GITHUB_PATH }
    }
}
if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) { Write-Output 'FAIL: no clang-cl (install the VS "C++ Clang tools for Windows" component)'; exit 1 }
Write-Output "clang-cl: $((Get-Command clang-cl).Source)"
clang-cl --version
cl 2>&1 | Select-Object -First 1
cmake --version | Select-Object -First 1
exit 0
