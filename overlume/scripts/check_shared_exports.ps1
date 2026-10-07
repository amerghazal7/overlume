# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_shared_exports.ps1 -Dll overlume.dll [-Dumpbin dumpbin.exe]
# Windows counterpart of check_shared_exports.sh: `dumpbin /exports` must list overlume:: symbols
# only (no filament / YAML / spdlog / Cesium / std names) and `dumpbin /dependents` only OS and
# C/C++ runtime DLLs. -ExportsText / -DependentsText feed canned dumpbin output (the selftest).
# Prints PASS or FAIL: ...; exit 0 / 1.
param(
    [Parameter(Mandatory = $true)][string]$Dll,
    [string]$Dumpbin = 'dumpbin',
    [string]$ExportsText,
    [string]$DependentsText
)
$ErrorActionPreference = 'Stop'

function Get-Text([string]$Canned, [string]$Switch) {
    if ($Canned) { return (Get-Content -Raw -LiteralPath $Canned) }
    $out = (& $Dumpbin /nologo $Switch $Dll | Out-String)
    if ($LASTEXITCODE -ne 0) { Write-Output "FAIL: dumpbin $Switch failed on $Dll"; exit 1 }
    return $out
}

$exports = @()
foreach ($line in (Get-Text $ExportsText '/exports') -split "`r?`n") {
    # "    ordinal hint RVA      name"
    if ($line -match '^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]{8}\s+(\S+)') { $exports += $Matches[1] }
}
if ($exports.Count -eq 0) { Write-Output "FAIL: no exports found in $Dll"; exit 1 }
$bad = @($exports | Where-Object { $_ -notmatch '@overlume@@' -or $_ -match 'filament|YAML|spdlog|Cesium' })
if ($bad.Count -gt 0) {
    Write-Output "FAIL: non-overlume exports:"
    $bad | Select-Object -First 20 | ForEach-Object { Write-Output "  $_" }
    exit 1
}

$allowed = '^(kernel32|user32|gdi32|opengl32|advapi32|shell32|ws2_32|bcrypt|crypt32|ole32|oleaut32|secur32|iphlpapi|winmm|ntdll|ucrtbase|rpcrt4|userenv|version|shlwapi|dbghelp|msvcp140[a-z_0-9]*|vcruntime140[a-z_0-9]*|concrt140|api-ms-win-[a-z0-9-]+)\.dll$'
$deps = @()
foreach ($line in (Get-Text $DependentsText '/dependents') -split "`r?`n") {
    if ($line -match '^\s+(\S+\.dll)\s*$') { $deps += $Matches[1] }
}
Write-Output ("DEPENDENTS: " + ($deps -join ' '))
$leaked = @($deps | Where-Object { $_.ToLowerInvariant() -notmatch $allowed })
if ($leaked.Count -gt 0) { Write-Output ("FAIL: unexpected dependencies: " + ($leaked -join ' ')); exit 1 }
Write-Output "PASS"
exit 0
