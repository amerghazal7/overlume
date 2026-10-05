# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# ensure_tools.ps1 -- installs (Chocolatey) whichever of ninja, makensis (NSIS), 7z and git the runner
# image lacks, and prints what is used. The x64 image carries all four; the arm64 image may not.
$ErrorActionPreference = 'Stop'
$want = @(@('ninja', 'ninja', ''), @('makensis', 'nsis', 'C:\Program Files (x86)\NSIS'), @('7z', '7zip', 'C:\Program Files\7-Zip'), @('git', 'git', ''))
foreach ($w in $want) {
    $cmd, $pkg, $dir = $w
    if ($dir -and (Test-Path $dir) -and ($env:PATH -notlike "*$dir*")) { $env:PATH = "$dir;$env:PATH"; if ($env:GITHUB_PATH) { $dir | Out-File -Append -Encoding utf8 $env:GITHUB_PATH } }
    if (-not (Get-Command $cmd -ErrorAction SilentlyContinue)) {
        Write-Output "installing $pkg (no $cmd on this runner)"
        choco install $pkg -y --no-progress | Out-Null
        if ($dir -and (Test-Path $dir)) { $env:PATH = "$dir;$env:PATH"; if ($env:GITHUB_PATH) { $dir | Out-File -Append -Encoding utf8 $env:GITHUB_PATH } }
        Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1" -ErrorAction SilentlyContinue
        refreshenv | Out-Null
    }
    $c = Get-Command $cmd -ErrorAction SilentlyContinue
    if (-not $c) { Write-Output "FAIL: $cmd still missing after installing $pkg"; exit 1 }
    Write-Output "$cmd -> $($c.Source)"
}
exit 0
