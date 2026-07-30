# v9.5 — Repository Cleanup

## What Changed

Over 9 major versions (v1 through v9.4), the repository had accumulated a massive amount of cruft. Build artifacts from ESP-IDF (`*.o`, `*.a`, `*.elf`, `*.bin`), Python `__pycache__` directories, VSCode settings, log files, and old version snapshots that referenced files that no longer existed. The total repo size had ballooned to over 2 GB on disk (most of it in `esp/build/` directories that should never have been committed).

The cleanup involved:

1. **Updated `.gitignore`** — Added comprehensive ignore patterns for build artifacts, cache directories, IDE files, and log directories.
2. **`git rm --cached`** — Removed tracked build artifacts from git without deleting the files locally. This is critical — the files stay on disk for local builds, but git stops tracking them.
3. **Verified with `git status --porcelain`** — After each removal pass, I checked the clean status to ensure nothing was missed.
4. **Organised root folder** — Removed stray files from `F:\WRO\World-Robot-Olympiad-2026\` (a few `.log` files, `sdkconfig.old` from misconfigured builds, `.a` library files that leaked from the cross-compiler).
5. **Pruned version snapshots** — Some `v1.x/` directories were empty or contained only obsolete docs. I kept the structure but removed orphaned files.

## Errors Encountered and Fixed

**Error 1: `git rm --cached` misses nested files.**
I ran `git rm --cached esp/build/*.o` to remove tracked `.o` files from the build directory. But the build directory has nested subdirectories (e.g., `esp/build/CMakeFiles/`, `esp/build/bootloader/`), and `*.o` files exist at multiple levels. The glob pattern `esp/build/*.o` only matches files directly in `esp/build/`, not in subdirectories.

The specific git output was:
```
$ git rm --cached esp/build/*.o
fatal: pathspec 'esp/build/*.o' did not match any files
```
But `git status --porcelain` still showed untracked `.o` files from subdirectories.

**Fix:** I used `git rm --cached -r esp/build/` to recursively remove everything under `esp/build/` from tracking. Then I re-added only the essential files (like `CMakeLists.txt`, `sdkconfig`, `Makefile`) that should be tracked. The `.gitignore` pattern `**/esp/build/` now prevents future artifacts from being tracked.

**Error 2: `git status --porcelain` still shows dirty files after cleanup.**
After running `git rm --cached` and updating `.gitignore`, `git status --porcelain` was still showing modified files. It turned out that `.gitignore` doesn't affect files that are already tracked — you MUST run `git rm --cached` first, THEN update `.gitignore`.

The sequence matters:
1. `git rm --cached <pattern>` — Stop tracking the file
2. Add pattern to `.gitignore` — Prevent re-tracking
3. `git status --porcelain` — Verify clean

**Error 3: Removing tracked `.bin` files broke the CI build.**
The CI pipeline (`build-esp` job) cached the `esp/build/` directory. When I removed `*.bin` from tracking and added them to `.gitignore`, the CI cache still had the old files, so size wasn't immediately reduced. But after the first CI run on the cleaned branch, the cache was rebuilt without the artifacts.

**Fix:** I cleared the GitHub Actions cache for the repository after confirming the new `.gitignore` was correct. The next CI run rebuilt the cache from scratch.

## Alternatives Considered

1. **`git filter-branch` to rewrite history.** This would completely remove build artifacts from git history, permanently shrinking the repository. But it rewrites every commit SHA, which would break all open PRs and make the history non-linear. Too risky for a competition project with multiple collaborators.

2. **BFG Repo-Cleaner.** BFG is faster than `git filter-branch` but has the same SHA-rewriting problem. I decided to leave the artifacts in the history (they're minor blips) and only clean from HEAD forward.

3. **Sparse checkout.** I considered using Git's sparse checkout feature so developers only clone the files they need. But this adds complexity to the clone process and judges would see an incomplete tree.

## Cleanup Result
- **Before:** ~2.1 GB repository (49% build artifacts, 23% VSCode/IDE files, 18% Python cache, 10% other)
- **After:** ~89 MB (mostly source code, docs, and config files)
- **`.gitignore` patterns added:** `**/esp/build/`, `**/__pycache__/`, `*.pyc`, `*.pyo`, `venv/`, `.venv/`, `logs/`, `*.bin`, `*.elf`, `*.map`, `*.o`, `*.a`, `.vscode/`, `sdkconfig.old`, `**/build/`
