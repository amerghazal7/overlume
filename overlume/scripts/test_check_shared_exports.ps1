# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Selftest of check_shared_exports.ps1 on canned dumpbin output (any host with PowerShell):
# a clean DLL must PASS; a leaked YAML export and a leaked dependency must each FAIL.
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $PSCommandPath
$ps = (Get-Process -Id $PID).Path
$w = Join-Path ([IO.Path]::GetTempPath()) ("ovl-exports-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $w | Out-Null
try {
    $okExports = @"
File Type: DLL
  Section contains the following exports for overlume.dll
    ordinal hint RVA      name
          1    0 000A1B2C ?create_renderer@overlume@@YAPEAVVisualRenderer@1@AEBURenderConfig@1@@Z
          2    1 000A1C00 ?render_frame@overlume@@YA_NPEAVVisualRenderer@1@@Z
"@
    $badExports = $okExports + "`n          3    2 000B0000 ?load@Node@YAML@@QEAAXXZ`n"
    $okDeps = "Image has the following dependencies:`n`n    KERNEL32.dll`n    OPENGL32.dll`n    VCRUNTIME140.dll`n    api-ms-win-crt-runtime-l1-1-0.dll`n"
    $badDeps = $okDeps + "    yaml-cpp.dll`n"
    Set-Content -LiteralPath "$w\ok.exports" -Value $okExports
    Set-Content -LiteralPath "$w\bad.exports" -Value $badExports
    Set-Content -LiteralPath "$w\ok.deps" -Value $okDeps
    Set-Content -LiteralPath "$w\bad.deps" -Value $badDeps
    function Run([string]$exp, [string]$dep) {
        & $ps -NoProfile -ExecutionPolicy Bypass -File "$here\check_shared_exports.ps1" -Dll x.dll `
            -ExportsText "$w\$exp" -DependentsText "$w\$dep" | Out-Null
        return $LASTEXITCODE
    }
    if ((Run 'ok.exports' 'ok.deps') -ne 0) { Write-Output 'FAIL: clean DLL rejected'; exit 1 }
    if ((Run 'bad.exports' 'ok.deps') -eq 0) { Write-Output 'FAIL: leaked YAML export accepted'; exit 1 }
    if ((Run 'ok.exports' 'bad.deps') -eq 0) { Write-Output 'FAIL: leaked dependency accepted'; exit 1 }
    Write-Output 'PASS: check_shared_exports.ps1'
    exit 0
}
finally {
    Remove-Item -Recurse -Force -LiteralPath $w -ErrorAction SilentlyContinue
}
