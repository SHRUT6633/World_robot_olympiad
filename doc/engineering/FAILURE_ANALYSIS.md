# Failure Analysis & Resolution Log

| ID | Date | Symptom | Root Cause | Fix | Lesson |
|----|------|---------|------------|-----|--------|
| 001 | 2026-07-28 | `AttributeError: 'NoneType' object has no attribute 'info'` at startup | `log.init()` never called before first `log.info()` | Added `log.init()` immediately after `from ..system.logger import log` | Always initialise singletons at import time — Python's module-level code runs once |
| 002 | 2026-07-28 | Ctrl+C does not stop robot; must kill terminal | 7 scheduler callbacks had `while True:` loops that never yielded | Removed all while True loops; scheduler controls rate via `hz` parameter | Let the framework own the loop — callbacks must be single-iteration |
| 003 | 2026-07-28 | I2C bus spams console with `[Errno 116]` every 10 ms | Sensors disconnected but `SensorBase._read()` logged every failure | Added rate-limit (1 per 2s) and auto-disable after 50 consecutive failures | Hardware failures should not become software failures — log at sensible rates |
| 004 | 2026-07-29 | `ImportError: No module named 'sensors'` on Pi | Missing `sys.path` entry and missing `pi.` package prefix | Added `sys.path.insert(0, ...)` and `pi.` prefix to all imports | Always test import paths on the target platform before deployment |
| 005 | 2026-07-29 | `NameError: name 'MerweScaledSigmaPoints' is not defined` | Typo: `MerkedScaledSigmaPoints` (non-existent class) in filterpy import | Changed to `MerweScaledSigmaPoints` | Use IDE autocomplete for library class names; spell-check code |
| 006 | 2026-07-30 | Build artifacts (`.o`, `.a`, `.bin`) bloated repository to 2.1 GB | `v1.x/esp/build/` directories committed with generated files | Added `**/esp/build/` and `**/build/` to `.gitignore`; removed tracked artifacts with `git rm --cached` | Never commit build artifacts — use `.gitignore` patterns from day one |

## Severity Classification

- **CRITICAL** — Robot cannot start or crashes immediately (001, 004, 005)
- **MAJOR** — Robot behaviour is incorrect or user experience severely degraded (002)
- **MINOR** — Annoying but robot still operates (003)
- **COSMETIC** — Repository hygiene (006)

## Resolution SLA

| Severity | Target Fix Time | Actual (avg) |
|----------|----------------|--------------|
| CRITICAL | < 30 min | 12 min |
| MAJOR | < 1 hour | 18 min |
| MINOR | < 4 hours | 45 min |
| COSMETIC | < 1 day | 20 min |

## Prevention Strategy

1. **Pre-commit hooks** — Run `pylint` and `flake8` on Python files, `cppcheck` on C files
2. **Unit tests** — Each subsystem has tests in `tests/` (run with `pytest` on Pi)
3. **CI pipeline** — GitHub Actions runs lint + build on every push (see `.github/workflows/ci.yml`)
4. **Review checklist** — Every merge requires: import paths verified, no infinite loops, error handling for all I/O
