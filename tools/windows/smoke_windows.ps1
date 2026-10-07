# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# smoke_windows.ps1 -Prefix C:\overlume -Arch x64|arm64 -- clean-room consumer of an INSTALLED Windows
# Overlume: builds tools/package_smoke against the prefix (find_package, the consumer's own vcpkg
# yaml-cpp, theme_assets_dir = nullptr) with the shared component and with the static one, then runs
# both. First runs --expect-no-gpu BEFORE Mesa is installed (System32 GDI GL 1.1: the platform_wgl
# probe must return nullptr), then renders a frame with Mesa llvmpipe beside the exe (x64 and arm64).
# The --expect-no-gpu/::warning:: render fallback is only for an arch without a Mesa build.
# Needs the Visual Studio environment (tools/windows/vsenv.ps1), cmake, ninja and vcpkg.
param(
    [Parameter(Mandatory = $true)][string]$Prefix,
    [Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch
)
$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$w = Join-Path ([IO.Path]::GetTempPath()) ("ovl-smoke-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $w | Out-Null
function Fail([string]$m) { Write-Output "FAIL: $m"; exit 1 }
function Native([string]$what, [scriptblock]$cmd) { & $cmd; if ($LASTEXITCODE -ne 0) { Fail "$what (exit $LASTEXITCODE)" } }

# The consumer's own yaml-cpp (static, /MD) must coexist with the one baked into overlume.dll.
$triplet = "$Arch-windows-static-md"
$vcpkgRoot = $env:VCPKG_INSTALLATION_ROOT
if (-not $vcpkgRoot -or -not (Test-Path (Join-Path $vcpkgRoot 'vcpkg.exe'))) {
    # The arm64 runner image may not carry vcpkg: use the one cesium-native's ezvcpkg bootstrapped, else clone one.
    $vcpkgRoot = Get-ChildItem -Path $env:EZVCPKG_BASEDIR -Directory -ErrorAction SilentlyContinue |
        Where-Object { Test-Path (Join-Path $_.FullName 'vcpkg.exe') } | Select-Object -ExpandProperty FullName -First 1
}
if (-not $vcpkgRoot) {
    $vcpkgRoot = Join-Path ([IO.Path]::GetTempPath()) 'vcpkg'
    Native 'clone vcpkg' { git clone --depth 1 https://github.com/microsoft/vcpkg $vcpkgRoot | Out-Null }
    Native 'bootstrap vcpkg' { & "$vcpkgRoot\bootstrap-vcpkg.bat" -disableMetrics | Out-Null }
}
$vcpkg = Join-Path $vcpkgRoot 'vcpkg.exe'
Native 'vcpkg install yaml-cpp' { & $vcpkg install "yaml-cpp:$triplet" --clean-after-build | Out-Null }
$yaml = Join-Path $vcpkgRoot "installed\$triplet"

$common = @('-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_COMPILER=cl', '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL',
            "-DCMAKE_PREFIX_PATH=$Prefix;$yaml")
Native 'configure shared consumer' { cmake -S "$repo\tools\package_smoke" -B "$w\shared" @common | Out-Null }
Native 'build shared consumer' { cmake --build "$w\shared" | Out-Null }
Native 'configure static consumer' { cmake -S "$repo\tools\package_smoke" -B "$w\static" @common -DSMOKE_STATIC=ON | Out-Null }
Native 'build static consumer' { cmake --build "$w\static" | Out-Null }

# overlume.dll is found through PATH (the installer's optional PATH entry, simulated).
$env:PATH = "$Prefix\bin;$env:PATH"
# No-GPU probe check (platform_wgl.cpp): dirs hold no opengl32.dll yet, so System32's GDI OpenGL 1.1 is in use and
# create_renderer() must return nullptr (not crash). Fails if the probe regresses to usable = true.
foreach ($exe in "$w\shared\package_smoke.exe", "$w\static\package_smoke_static.exe") {
    $o = & $exe --expect-no-gpu 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) { Write-Output $o; Fail "$(Split-Path -Leaf $exe) --expect-no-gpu before Mesa ($Arch): platform_wgl probe regressed" }
}
Write-Output "PASS: pre-Mesa --expect-no-gpu ($Arch): platform_wgl probe returns nullptr without OpenGL 4.1"
$mesa = $false
foreach ($d in "$w\shared", "$w\static") {
    $mo = & (Join-Path $PSScriptRoot 'install_mesa.ps1') -Arch $Arch -Dest $d
    if ($LASTEXITCODE -eq 0) { $mesa = $true }
    elseif ($LASTEXITCODE -ne 2) { Write-Output $mo; Fail "Mesa llvmpipe install ($Arch) failed" }
}
$env:GALLIUM_DRIVER = 'llvmpipe'
$exes = @("$w\shared\package_smoke.exe", "$w\static\package_smoke_static.exe")

$gpu = 'render'
$out = & $exes[0] --expect-render 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) {
    if ($out -match 'create_renderer returned nullptr') {
        if ($mesa) { Write-Output $out; Fail "create_renderer returned nullptr on $Arch with Mesa llvmpipe" }
        $gpu = 'nogpu'
        Write-Output "::warning::no OpenGL 4.1 context on this runner ($Arch$(if (-not $mesa) { ', no Mesa llvmpipe build for this architecture' })): smoke ran --expect-no-gpu, no frame was rendered"
    }
    else { Write-Output $out; Fail "shared smoke ($Arch)" }
}
$flag = if ($gpu -eq 'render') { '--expect-render' } else { '--expect-no-gpu' }
foreach ($exe in $exes) {
    $o = & $exe $flag 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) { Write-Output $o; Fail "$(Split-Path -Leaf $exe) $flag ($Arch)" }
}
Remove-Item -Recurse -Force $w -ErrorAction SilentlyContinue
Write-Output "PASS: Windows smoke ($Arch, $gpu): shared find_package + static"
exit 0
