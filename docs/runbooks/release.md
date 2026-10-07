# Release runbook

How to cut, verify, repair and rotate the keys of an Overlume release. The
design and the per-task record are in
[`plans/2026-10-02-cross-platform-release.md`](../plans/2026-10-02-cross-platform-release.md);
what users run to install is the README's "Install" section. Credentials are
named here, never shown: values live only in the repository's Actions secrets
and in `~/.config/overlume-release/` (mode 700) on the maintainer's machine.

A tag push (`v*`) runs `.github/workflows/release.yml`. Nothing is public until
every platform has built, tested and signed: the release is created as a
**draft**, the publishers (Homebrew tap, SwiftPM repo, Maven Central) run only
after the merged, signed `SHA256SUMS` exists, and a final `finalize` job flips
the draft to published and dispatches `pages.yml` on `main` (apt/yum repos and
API docs). Any failure before `finalize` leaves a draft and the Linux/Windows/
Android/Apple channels untouched.

## Prerequisites (one-off)

| What | Where | Check |
|---|---|---|
| GPG release key (RSA-4096, fingerprint in the README, expires 2029-10-01) | secrets `OVERLUME_GPG_PRIVATE_KEY`, `OVERLUME_GPG_PASSPHRASE`, `OVERLUME_GPG_FINGERPRINT`; public half `packaging/keys/overlume-release.asc` | `pages.yml` fails if the served key's fingerprint differs from the secret |
| Maven Central namespace `io.github.amerghazal7` verified, publisher token | secrets `MAVEN_CENTRAL_USERNAME`, `MAVEN_CENTRAL_PASSWORD` (Central Portal user token; the script builds the Bearer value) | dry runs run `publish-maven` in validate mode (upload, wait for `VALIDATED`, drop) |
| Public key on keyservers: Central checks signatures against `keyserver.ubuntu.com` and `keys.openpgp.org` | `curl -s -o /dev/null -w '%{http_code}\n' "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x<fingerprint>"` must print 200 (keys.openpgp.org: `https://keys.openpgp.org/vks/v1/by-fingerprint/<fingerprint>`) | `keys.openpgp.org` shows the UID only after the emailed link is clicked; keyserver.ubuntu.com propagation can lag a day |
| Homebrew tap `amerghazal7/homebrew-overlume`, SwiftPM repo `amerghazal7/overlume-swift` | write deploy keys as secrets `HOMEBREW_TAP_DEPLOY_KEY`, `SWIFTPM_REPO_DEPLOY_KEY` (public halves registered as deploy keys with write access) | `tools/release/test_apple_publish.sh` runs both publishers against local bare repos |
| GitHub Pages source = GitHub Actions, environment `github-pages` allows branch `main` | repo settings | first real deploy on `main` |
| Apple signing (optional, see below), Windows signing (optional, see below) | secrets named below | without them the jobs emit `::warning::` and ship unsigned |

## Cutting a release

1. **Rehearse.** On the work branch (or `main`), run a full dry run and, once,
   the pages dry run, then smoke the channels locally against them:

   ```bash
   gh workflow run release.yml -R amerghazal7/overlume --ref <branch> -f dry_run=true
   gh workflow run pages.yml   -R amerghazal7/overlume --ref <branch> -f dry_run=true
   gh run list -R amerghazal7/overlume --limit 4            # note both run ids, wait for success
   tools/release/channel_smoke.sh all <RELEASE_RUN_ID> <PAGES_RUN_ID>
   ```

   A dry run builds, tests and signs everything with the real key, validates
   and drops the Maven bundle, publishes nothing and skips `create`,
   `publish-homebrew`, `publish-swiftpm`, `finalize`, `pages`. Its artifacts
   (`pkg-*`, `maven-bundle`, `release-sums`, `channels-local`) expire after the
   repo's retention period, so smoke soon after. `channel_smoke.sh all` covers
   Linux (sums + signature, tar.gz, apt, dnf, vcpkg, Conan shared + static) and
   Android (NDK consumer, Prefab, Maven bundle signatures); Windows, macOS and
   iOS install paths are exercised by the CI jobs `package-windows*`,
   `package-macos`, `package-ios`, `channels`.

2. **Bump the version** in both places (they must agree; CMake fails the
   configure on drift and the `create` job fails on a tag mismatch):
   `overlume/include/overlume/version.h` (`OVERLUME_VERSION_MAJOR/MINOR/PATCH`)
   and `project(overlume VERSION x.y.z ...)` in `overlume/CMakeLists.txt`.
   Public struct changes also bump `kSceneVersion` (ADR-0004); free functions
   bump nothing.

3. **CHANGELOG.** Rename the `## [Unreleased]` heading's content into
   `## [x.y.z] - YYYY-MM-DD` (keep a fresh empty `[Unreleased]` above it). The
   `create` job extracts that section as the release notes and fails if it is
   missing or empty.

4. **Gate and commit.** `tools/ci_visual_mode.sh` green (foreground), then
   commit `release: vX.Y.Z` and merge to `main`.

5. **Tag and push** from the merged commit:

   ```bash
   git tag -a vX.Y.Z -m "Overlume X.Y.Z" && git push origin vX.Y.Z
   ```

6. **Watch.** `gh run watch -R amerghazal7/overlume $(gh run list -R amerghazal7/overlume --workflow release.yml --limit 1 --json databaseId -q '.[0].databaseId')`.
   Expect: `create` (draft) -> package jobs for Linux x86_64/aarch64, Android,
   macOS, iOS, Windows x64/arm64 -> `channels-render` -> `sums` (merges the
   per-job signed checksums, signs `SHA256SUMS.asc`, uploads to the draft) ->
   `channels` (vcpkg/Conan on three OSes) -> `publish-homebrew`,
   `publish-swiftpm` -> `publish-maven` (irreversible, last) -> `finalize` ->
   `pages`.

7. **Verify what users get** (after `finalize`; the Homebrew/SwiftPM pushes
   precede publication, so their asset URLs go live seconds later):

   - Release assets: packages for every platform, `overlume-X.Y.Z-vcpkg-port.zip`,
     `overlume-X.Y.Z-conan-recipe.zip`, `SHA256SUMS`, `SHA256SUMS.asc`. Download
     and run the README's "Manual archives, verified" block.
   - apt/yum: after `pages.yml` finishes on `main`,
     `curl -fsSL https://amerghazal7.github.io/overlume/overlume.repo` and
     `.../apt/dists/stable/InRelease` (200, `gpg --verify` with the committed
     key); `tools/release/channel_smoke.sh repo <extracted site>` also works
     against a downloaded `github-pages` artifact.
   - Homebrew: `brew install amerghazal7/overlume/overlume && brew test overlume`
     on a Mac.
   - SwiftPM: `git ls-remote --tags https://github.com/amerghazal7/overlume-swift vX.Y.Z`;
     resolve the package in a scratch Xcode/SwiftPM project.
   - Maven: `https://repo1.maven.org/maven2/io/github/amerghazal7/overlume/X.Y.Z/`
     lists `.aar`, `.pom`, `.asc` (propagation to search.maven.org takes hours;
     the repo1 directory is the authoritative check).
   - Docs: `curl -s -o /dev/null -w '%{http_code}' https://amerghazal7.github.io/overlume/` prints 200.

## Repairing a failed run

- **Re-run only the failed platform:** `gh run rerun <run-id> -R amerghazal7/overlume --failed`
  (or the "Re-run failed jobs" button). Uploads go to the draft with `--clobber`,
  so reruns replace their own assets only. The `sums` upload refuses a release
  that is no longer a draft; the SwiftPM publisher refuses a rebuilt zip for a
  version already pinned (a version is write-once); the Maven publisher exits 0
  when Central already has the version. If an earlier Maven deployment is still
  `PUBLISHING`, wait for it to land, then re-run.
- **A published release that is wrong:** do not edit assets. Delete the tag and
  release only if nothing has consumed them (Maven Central versions can never
  be removed), otherwise ship `X.Y.Z+1`.
- **Draft stuck, publishers skipped:** that is the design after a package
  failure; fix, rerun failed jobs, `finalize` then runs.

## Key rotation

One GPG key signs rpm packages, the apt/yum repositories, Maven artifacts and
`SHA256SUMS`. Rotate on compromise or before it expires (2029-10-01).

1. Generate a new RSA-4096 key (`gpg --full-generate-key`), a strong passphrase,
   and a revocation certificate; keep all three under `~/.config/overlume-release/`.
2. Update the three secrets: `OVERLUME_GPG_PRIVATE_KEY` (armored, `gpg --armor
   --export-secret-keys <fpr>`), `OVERLUME_GPG_PASSPHRASE`, `OVERLUME_GPG_FINGERPRINT`
   (`gh secret set NAME -R amerghazal7/overlume < file`; never echo them).
3. Replace `packaging/keys/overlume-release.asc` with the new public key and
   update the fingerprint in the README "Install" section. (`channel_smoke.sh`
   and `merge_sums.sh` read the committed key, not a hard-coded fingerprint.)
4. Upload the public key to `keyserver.ubuntu.com` and `keys.openpgp.org`
   (confirm the emailed link), re-check Prerequisites above.
5. Dry-run release + pages, then `channel_smoke.sh all`. `pages.yml` fails
   unless the served key equals `OVERLUME_GPG_FINGERPRINT`.
6. Publish the new release; `pages` re-signs the apt/yum repos with the new key
   and serves the new `overlume-release.asc`. Existing users re-import it:
   `sudo curl -fsSL https://amerghazal7.github.io/overlume/overlume-release.asc -o /etc/apt/keyrings/overlume.asc`
   (apt), accept the new key at the next `dnf` prompt (or `sudo rpm --import
   <url>`), and `gpg --import` for archive verification. Announce the change in
   the release notes. On compromise, publish the revocation certificate too.
7. Maven Central keeps accepting old releases; new ones need the new key on the
   keyservers first.

## Provisioning signing secrets (until then the jobs warn and ship unsigned)

**Apple** (macOS `.pkg`/dylib and iOS xcframework; `tools/apple/sign_and_notarize.sh`):
in the Apple Developer account create a *Developer ID Application* and a
*Developer ID Installer* certificate, export both into one `.p12` and set:

```bash
base64 -w0 DeveloperID.p12 | gh secret set APPLE_DEVELOPER_ID_P12 -R amerghazal7/overlume
gh secret set APPLE_DEVELOPER_ID_P12_PASSWORD -R amerghazal7/overlume   # prompts
gh secret set APPLE_NOTARY_KEY_ID    -R amerghazal7/overlume            # App Store Connect API key id
gh secret set APPLE_NOTARY_ISSUER_ID -R amerghazal7/overlume
gh secret set APPLE_NOTARY_KEY_P8    -R amerghazal7/overlume < AuthKey_XXXX.p8
```

Then a dry run must show `codesign`, `productsign`, `notarytool ... Accepted`,
`stapler validate` in `package-macos` and no "Apple signing secrets not
configured" warning. Notarisation applies to the `.pkg`; the Homebrew tarball
is code-signed (hardened runtime) only.

**Windows** (`tools/windows/sign.ps1`, Authenticode on `overlume.dll` and the
NSIS installers): a code-signing certificate as `.pfx`:

```bash
base64 -w0 codesign.pfx | gh secret set WINDOWS_SIGNING_PFX_BASE64 -R amerghazal7/overlume
gh secret set WINDOWS_SIGNING_PFX_PASSWORD -R amerghazal7/overlume
```

(Azure Trusted Signing is not implemented.) A dry run's `sign-windows` then runs
`signtool verify /pa` instead of the "unsigned" warning. Until both exist,
SmartScreen and Gatekeeper warn on the installers; the GPG-signed `SHA256SUMS.asc`
still covers integrity.

## The real-Mac Metal check (before the first tag)

Hosted macOS runners have a paravirtual GPU that cannot drive Filament, so no
Metal frame has ever rendered in CI (the gpu ctest label, the macOS smoke
render and the iOS simulator render are skipped with `::warning::`; open item
13 in [`../status.md`](../status.md)). On a real Mac (arm64 and, if available,
x86_64 or Rosetta), from a checkout of the release commit:

```bash
# download the dry run's pkg-macos artifact (overlume-X.Y.Z-macos-universal.tar.gz) and unpack it to <prefix>, or build per README
ctest --test-dir overlume/build -L gpu --output-on-failure   # includes ReadbackOrientation.Row0IsTopOfImage (build natively on the Mac)
tools/apple/smoke_macos.sh <prefix> arm64           # renders a frame, expects PASS
```

Record the result (date, hardware, macOS version) in `docs/status.md` item 13
and close it. Do not tag before this is done.

## Merge checklist (release-packaging -> main)

- [ ] Remove the temporary `push: branches: [release-packaging]` trigger from
  `.github/workflows/pages.yml` (it only registers the workflow for dispatch;
  deploy is guarded to `main`).
- [x] `finalize` dispatches `pages.yml` on `main` (route b); `workflow_call` is gone from `pages.yml`.
- [ ] After the first push to `main` deploys, `curl -s -o /dev/null -w '%{http_code}'` on the live docs index prints 200.
- [ ] Real-Mac Metal check done (above), `docs/status.md` item 13 closed.
- [ ] Open items owned by the user closed or consciously carried: keyserver
  propagation, Apple and Windows signing secrets, the m2o1 real-robot lidar
  calibration (still unverified; unrelated to packaging).
- [ ] First real tag: a `vX.Y.Z` run end to end (draft -> published, `pages` on
  `main`), then the "Verify what users get" list above.

## Known limits (carried in the README and `docs/status.md`)

Linux packages need glibc >= 2.28; the Linux static component needs clang and
libc++ >= 18 (Ubuntu 22.04 and EL9 have none: unsupported for static); Windows
arm64 is built with MSVC v145, so arm64 static consumers need a VS 2026 linker
and VC runtime >= 14.51 (x64 static: VS 2022 >= 17.14); the Linux aarch64
packages are cross-built and their native tests run on the `ubuntu-22.04-arm`
runner in CI only.
