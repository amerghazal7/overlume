# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# install_mesa.ps1 -Arch x64|arm64 -Dest DIR -- puts Mesa's llvmpipe OpenGL (opengl32.dll +
# libgallium_wgl.dll, pal1000/mesa-dist-win, pinned release, SHA256-checked) next to the test
# executables in DIR, so Filament's WGL back end renders in software on a GPU-less runner (a DLL beside
# the exe wins over System32's GDI OpenGL 1.1). Exit 0 = installed; exit 2 = no build for this arch
# (mesa-dist-win publishes x64 and x86 only), the caller reports the --expect-no-gpu path.
param(
    [Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch,
    [Parameter(Mandatory = $true)][string]$Dest
)
$ErrorActionPreference = 'Stop'
$version = '26.2.4'
$sha256 = '351fc8c8b695878ffb3eaa044b3ead08672a48b1a045e3c3e3975811df0f6695'
if ($Arch -ne 'x64') {
    Write-Output "no Mesa llvmpipe build for Windows $Arch (pal1000/mesa-dist-win ships x64 and x86 only)"
    exit 2
}
$tmp = Join-Path ([IO.Path]::GetTempPath()) "mesa-$version"
New-Item -ItemType Directory -Force -Path $tmp | Out-Null
$archive = Join-Path $tmp "mesa3d-$version-release-msvc.7z"
if (-not (Test-Path $archive)) {
    Invoke-WebRequest -UseBasicParsing -Uri "https://github.com/pal1000/mesa-dist-win/releases/download/$version/mesa3d-$version-release-msvc.7z" -OutFile $archive
}
$got = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
if ($got -ne $sha256) { Remove-Item -Force $archive; Write-Output "FAIL: Mesa archive SHA256 mismatch ($got)"; exit 1 }
7z x -y "-o$tmp" $archive 'x64\opengl32.dll' 'x64\libgallium_wgl.dll' | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL: 7z extraction failed'; exit 1 }
New-Item -ItemType Directory -Force -Path $Dest | Out-Null
Copy-Item -Force "$tmp\x64\opengl32.dll", "$tmp\x64\libgallium_wgl.dll" $Dest
Write-Output "PASS: Mesa $version llvmpipe (x64) installed in $Dest"
exit 0
