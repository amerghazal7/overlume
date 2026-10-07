# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# sign.ps1 -Path file [file ...] [-TestMode]
# Authenticode-signs the given files (overlume.dll, the NSIS installer) when the signing secrets are
# present in the environment, otherwise emits a ::warning:: and exits 0 (unsigned packages; never a
# silent skip). Secrets are referenced by name only and never printed.
#   WINDOWS_SIGNING_PFX_BASE64    base64 of the code-signing .pfx
#   WINDOWS_SIGNING_PFX_PASSWORD  its password
# Signed files are checked with `signtool verify /pa`. -TestMode (CI selftest with a throw-away
# self-signed certificate) skips the RFC 3161 timestamp and checks only that the signature is present.
param(
    [Parameter(Mandatory = $true, ValueFromRemainingArguments = $true)][string[]]$Path,
    [switch]$TestMode
)
$ErrorActionPreference = 'Stop'

$pfx64 = $env:WINDOWS_SIGNING_PFX_BASE64
$pw = $env:WINDOWS_SIGNING_PFX_PASSWORD
if ([string]::IsNullOrEmpty($pfx64) -or [string]::IsNullOrEmpty($pw)) {
    Write-Output "::warning::Windows signing secrets (WINDOWS_SIGNING_PFX_BASE64, WINDOWS_SIGNING_PFX_PASSWORD) not configured; unsigned: $($Path -join ', ')"
    exit 0
}

$signtool = Get-ChildItem -Path "${env:ProgramFiles(x86)}\Windows Kits\10\bin\*\*\signtool.exe" -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -match '\\(x64|arm64)\\' } | Sort-Object FullName -Descending | Select-Object -First 1
if (-not $signtool) { Write-Output 'FAIL: signtool.exe not found (Windows SDK)'; exit 1 }

$tmp = Join-Path ([IO.Path]::GetTempPath()) ("ovl-sign-" + [Guid]::NewGuid().ToString('N') + '.pfx')
try {
    [IO.File]::WriteAllBytes($tmp, [Convert]::FromBase64String($pfx64))
    foreach ($file in $Path) {
        $stArgs = @('sign', '/fd', 'sha256', '/f', $tmp, '/p', $pw)
        if (-not $TestMode) { $stArgs += @('/tr', 'http://timestamp.digicert.com', '/td', 'sha256') }
        $stArgs += $file
        & $signtool.FullName @stArgs | Out-Null
        if ($LASTEXITCODE -ne 0) { Write-Output "FAIL: signtool sign failed on $file"; exit 1 }
        if ($TestMode) {
            $sig = Get-AuthenticodeSignature -LiteralPath $file
            if ($null -eq $sig.SignerCertificate) { Write-Output "FAIL: $file carries no signature"; exit 1 }
        }
        else {
            & $signtool.FullName verify /pa $file | Out-Null
            if ($LASTEXITCODE -ne 0) { Write-Output "FAIL: signtool verify /pa failed on $file"; exit 1 }
        }
        Write-Output "PASS: signed $file"
    }
}
finally {
    Remove-Item -Force -LiteralPath $tmp -ErrorAction SilentlyContinue
}
exit 0
