# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_windows.ps1 build|test|package -Arch x64|arm64 -- the Windows legs of release.yml, so CI and a
# developer shell run the same commands. Needs the Visual Studio environment (tools/windows/vsenv.ps1)
# and ninja; `package` also needs NSIS (makensis) and prints PASS/FAIL only.
#   build    configure + build overlume/build-windows-ARCH (MSVC cl, Ninja, Release, /MD, Cesium ON)
#   test     ctest -L cpu; ctest -L gpu when an OpenGL 4.1 context exists (Mesa llvmpipe on x64)
#   package  sign overlume.dll, cpack NSIS+ZIP into overlume/pkg-windows-ARCH, sign the installer
param(
    [Parameter(Mandatory = $true, Position = 0)][ValidateSet('build', 'test', 'package')][string]$Phase,
    [Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$b = Join-Path $repo "overlume\build-windows-$Arch"
$pkg = Join-Path $repo "overlume\pkg-windows-$Arch"
function Fail([string]$m) { Write-Output "FAIL: $m"; exit 1 }
function Native([string]$what, [scriptblock]$cmd) { & $cmd; if ($LASTEXITCODE -ne 0) { Fail "$what (exit $LASTEXITCODE)" } }

switch ($Phase) {
    'build' {
        # CMake 4 (the runners') rejects the < 3.5 minimums of pinned third-party projects (yaml-cpp 0.8.0).
        $env:CMAKE_POLICY_VERSION_MINIMUM = '3.5'
        Native 'configure' {
            cmake -S "$repo\overlume" -B $b -G Ninja -DCMAKE_BUILD_TYPE=Release `
                -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl `
                -DOVERLUME_ENABLE_CESIUM=ON -DOVERLUME_BUILD_DOCS=OFF -DOVERLUME_BUILD_EXAMPLES=OFF
        }
        Native 'build' { cmake --build $b -- -k 0 }   # report every compile error, not just the first
        Write-Output "PASS: built $Arch ($b)"
    }
    'test' {
        # Software OpenGL for the render tests: Mesa llvmpipe beside the test executables.
        & (Join-Path $PSScriptRoot 'install_mesa.ps1') -Arch $Arch -Dest $b
        $rc = $LASTEXITCODE   # 0 installed, 2 no build for this arch (documented no-GPU path), 1 checksum/extract failure
        if ($rc -ne 0 -and $rc -ne 2) { Fail "Mesa llvmpipe install ($Arch): checksum/extract failed (exit $rc)" }
        $mesa = ($rc -eq 0)
        $env:GALLIUM_DRIVER = 'llvmpipe'
        # FiftyObjectsSceneUpdateUnderTwoMilliseconds is a wall-clock budget a shared runner cannot promise
        # (docs/status.md, known gap 11).
        # Run every group before failing, so one CI run shows all of the failures.
        $failed = @()
        ctest --test-dir $b -L cpu --output-on-failure --timeout 900 -E FiftyObjectsSceneUpdateUnderTwoMilliseconds
        if ($LASTEXITCODE -ne 0) { $failed += 'ctest -L cpu' }
        $probe = (ctest --test-dir $b -R '^HelloFrame\.RendersDistinctSkyAndGround$' -V | Out-String)
        if ($probe -match 'No GPU/EGL device') {
            # Only a missing Mesa build (arm64) may skip rendering; with Mesa installed a missing context is a regression.
            if ($mesa) { Fail "no OpenGL 4.1 context on $Arch despite Mesa llvmpipe (gpu tests incl. row-0-is-top would be skipped)" }
            $why = "no Mesa llvmpipe build exists for Windows $Arch"
            Write-Output "::warning::no OpenGL 4.1 context on this runner ($Arch): $why. gpu-labelled tests (incl. row-0-is-top orientation) not run; smoke runs --expect-no-gpu"
        }
        else {
            ctest --test-dir $b -L gpu --output-on-failure --timeout 900
            if ($LASTEXITCODE -ne 0) { $failed += 'ctest -L gpu' } else { Write-Output "PASS: gpu tests rendered on $Arch (Mesa llvmpipe)" }
        }
        if ($failed.Count -gt 0) { Fail ($failed -join ', ') }
    }
    'package' {
        $sign = Join-Path $PSScriptRoot 'sign.ps1'
        & $sign -Path "$b\overlume.dll"
        if ($LASTEXITCODE -ne 0) { Fail 'sign overlume.dll' }
        Remove-Item -Recurse -Force $pkg -ErrorAction SilentlyContinue
        Native 'cpack' { cpack --config "$b\CPackConfig.cmake" -B $pkg }
        $exe = @(Get-ChildItem "$pkg\overlume-*-windows-$Arch.exe")
        $zip = @(Get-ChildItem "$pkg\overlume-*-windows-$Arch.zip")
        if ($exe.Count -ne 1 -or $zip.Count -ne 1) { Fail "expected one installer and one zip in $pkg" }
        & $sign -Path $exe[0].FullName
        if ($LASTEXITCODE -ne 0) { Fail 'sign installer' }
        # The zip carries both components, prefix-relative.
        $names = & 7z l -ba $zip[0].FullName | ForEach-Object { ($_ -split '\s+', 6)[-1] -replace '\\', '/' }
        foreach ($want in 'bin/overlume.dll', 'lib/overlume.lib', 'lib/overlume_static.lib', 'include/overlume/api.h',
                          'lib/cmake/overlume/overlumeConfig.cmake', 'lib/cmake/overlume/overlumeStaticTargets.cmake') {
            if ($names -notcontains $want) { Fail "$($zip[0].Name) lacks $want" }
        }
        if (-not ($names | Where-Object { $_ -like 'share/overlume/themes/*.yaml' })) { Fail "$($zip[0].Name) lacks the themes" }
        Copy-Item -Force "$b\CPackConfig.cmake" $pkg -ErrorAction SilentlyContinue
        Get-ChildItem $pkg -File | ForEach-Object { Write-Output ("{0}  {1:N1} MB" -f $_.Name, ($_.Length / 1MB)) }
        Write-Output "PASS: packaged $Arch ($pkg)"
    }
}
exit 0
