# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# install_mesa.ps1 -Arch x64|arm64 -Dest DIR -- puts Mesa's llvmpipe OpenGL (opengl32.dll [+ libgallium_wgl.dll
# on x64]) next to the test executables in DIR, so Filament's WGL back end renders in software on a GPU-less
# runner (a DLL beside the exe wins over System32's GDI OpenGL 1.1). Pinned, SHA256-checked downloads:
#   x64:   pal1000/mesa-dist-win 26.2.4 (opengl32.dll + libgallium_wgl.dll)
#   arm64: mmozeiko/build-mesa 26.2.4 (statically linked, single opengl32.dll; the build mesa-dist-win's own
#          release notes designate for ARM64)
# Exit 0 = installed; exit 1 = checksum/extract failure. (Exit 2, "no build for this arch", is no longer used.)
param(
    [Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch,
    [Parameter(Mandatory = $true)][string]$Dest
)
$ErrorActionPreference = 'Stop'
$version = '26.2.4'
if ($Arch -eq 'x64') {
    $name = "mesa3d-$version-release-msvc.7z"
    $url = "https://github.com/pal1000/mesa-dist-win/releases/download/$version/$name"
    $sha256 = '351fc8c8b695878ffb3eaa044b3ead08672a48b1a045e3c3e3975811df0f6695'
    $dlls = 'opengl32.dll', 'libgallium_wgl.dll'
}
else {
    $name = "mesa-llvmpipe-arm64-$version.7z"
    $url = "https://github.com/mmozeiko/build-mesa/releases/download/$version/$name"
    $sha256 = '171e0cc3d48a2d435f7ebda28159d800bed7c4b2bda3ac7f9cc708b56f5499a1'
    $dlls = @('opengl32.dll')
}
$tmp = Join-Path ([IO.Path]::GetTempPath()) "mesa-$version"
New-Item -ItemType Directory -Force -Path $tmp | Out-Null
$archive = Join-Path $tmp $name
if (-not (Test-Path $archive)) { Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $archive }
$got = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
if ($got -ne $sha256) { Remove-Item -Force $archive; Write-Output "FAIL: Mesa archive SHA256 mismatch ($got)"; exit 1 }
$ex = Join-Path $tmp "x-$Arch"
Remove-Item -Recurse -Force $ex -ErrorAction SilentlyContinue
7z x -y "-o$ex" $archive | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL: 7z extraction failed'; exit 1 }
New-Item -ItemType Directory -Force -Path $Dest | Out-Null
foreach ($d in $dlls) {
    # x64 archive: x64\<dll>; arm64 archive layout is not assumed -- take the first match anywhere.
    $f = Get-ChildItem -Recurse -File -Path $ex -Filter $d | Where-Object { $Arch -ne 'x64' -or $_.DirectoryName -match '\\x64$' } | Select-Object -First 1
    if (-not $f) { Write-Output "FAIL: $d not found in $name"; exit 1 }
    Copy-Item -Force $f.FullName $Dest
}
Write-Output "PASS: Mesa $version llvmpipe ($Arch) installed in $Dest"
exit 0
