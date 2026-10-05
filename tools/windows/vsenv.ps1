# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# vsenv.ps1 -Arch x64|arm64 -- exports the Visual Studio (MSVC v143) build environment for the target
# architecture into $GITHUB_ENV (what vcvarsall.bat sets), and prints the tool versions. The build uses MSVC cl (Filament 1.56.5 refuses clang on Windows).
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

cl 2>&1 | Select-Object -First 1
cmake --version | Select-Object -First 1
exit 0
