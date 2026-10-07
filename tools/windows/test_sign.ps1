# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_sign.ps1 -- selftest of sign.ps1: without signing secrets it must warn (never skip silently)
# and leave the file unsigned; with a throw-away self-signed code-signing certificate in the secrets'
# place it must sign. Fails if either behaviour regresses.
$ErrorActionPreference = 'Stop'
$sign = Join-Path $PSScriptRoot 'sign.ps1'
$w = Join-Path ([IO.Path]::GetTempPath()) ("ovl-signtest-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $w | Out-Null
$cert = $null
try {
    $victim = Join-Path $w 'victim.dll'
    Set-Content -Path $victim -Value 'untouched: the no-secrets path must not open it' -Encoding ascii

    # 1. No secrets: warning on stdout, exit 0.
    Remove-Item Env:WINDOWS_SIGNING_PFX_BASE64, Env:WINDOWS_SIGNING_PFX_PASSWORD -ErrorAction SilentlyContinue
    $out = (& $sign -Path $victim | Out-String)
    if ($LASTEXITCODE -ne 0 -or $out -notmatch '::warning::Windows signing secrets') { Write-Output "FAIL: unsigned path did not warn: $out"; exit 1 }

    # 2. Secrets present (a self-signed test certificate): the file really gets signed.
    $pwPlain = [Guid]::NewGuid().ToString('N')
    $cert = New-SelfSignedCertificate -Type CodeSigningCert -Subject 'CN=overlume-sign-selftest' -CertStoreLocation Cert:\CurrentUser\My
    $pfx = Join-Path $w 't.pfx'
    Export-PfxCertificate -Cert $cert -FilePath $pfx -Password (ConvertTo-SecureString $pwPlain -AsPlainText -Force) | Out-Null
    $env:WINDOWS_SIGNING_PFX_BASE64 = [Convert]::ToBase64String([IO.File]::ReadAllBytes($pfx))
    $env:WINDOWS_SIGNING_PFX_PASSWORD = $pwPlain
    $target = Join-Path $w 'target.exe'
    Copy-Item (Join-Path $env:SystemRoot 'System32\whoami.exe') $target
    & $sign -Path $target -TestMode | Out-Null
    if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL: signed path failed'; exit 1 }
    # The PKCS#7 blob embeds the signer's subject in the clear.
    $bytes = [IO.File]::ReadAllBytes($target)
    if (-not [Text.Encoding]::Latin1.GetString($bytes).Contains('overlume-sign-selftest')) { Write-Output 'FAIL: file not signed with the test certificate'; exit 1 }
    Write-Output 'PASS: sign.ps1 (unsigned path warns, signed path signs)'
    exit 0
}
finally {
    Remove-Item Env:WINDOWS_SIGNING_PFX_BASE64, Env:WINDOWS_SIGNING_PFX_PASSWORD -ErrorAction SilentlyContinue
    if ($cert) { Remove-Item -LiteralPath "Cert:\CurrentUser\My\$($cert.Thumbprint)" -ErrorAction SilentlyContinue }
    Remove-Item -Recurse -Force -LiteralPath $w -ErrorAction SilentlyContinue
}
