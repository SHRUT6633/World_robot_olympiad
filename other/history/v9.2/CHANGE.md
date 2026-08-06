# v9.2 — Error Catalog

## What Changed

Every embedded system fails. The question isn't whether sensors will error — it's whether the robot keeps driving when they do. For the competition, we needed to prove to the judges that we understand every failure mode and have handled it explicitly.

I created `docs/issues/error_catalog.md` (later moved to `docs/issues/`), documenting every single error that can occur in the system. Each entry includes:

1. **Exact error message** — Copy-pasted from actual log output
2. **When it occurs** — Which line of code, which subsystem, which phase
3. **Root cause** — Why it happens, both hardware and software
4. **How the code handles it** — Exception caught? Fallback behaviour? Auto-recovery?
5. **Severity** — CRITICAL / HIGH / MEDIUM / LOW

The catalog covers 12 distinct errors:
- I2C bus errors (Errno 121 Remote I/O, Errno 5 Input/output)
- Camera init failures and frame drops
- UART SerialException (ESP32 not responding)
- UKF singular matrix in covariance update
- Logger not initialised (AttributeError at startup)
- Scheduler task timeout (callback never returns)
- Pillar not detected (lighting, FOV, thresholds)
- Pillar passed wrong side (config issue)
- Parking failure (wrong steering mode)
- Vehicle check failure (physical size/weight)
- CRC errors on UART packets
- Emergency stop activated

## Errors Encountered and Fixed

**Error 1: Some errors are impossible to reproduce (hardware dependent).**
I tried to test every error path by physically disconnecting sensors. For I2C sensors this was easy — unplug the VL53L0X and watch the error log. But for hardware-dependent errors like "UART buffer overflow at 921600 baud," I couldn't reproduce it because we run at 115200 with a tiny control loop.

**Fix:** Instead of testing, I did static code analysis. I traced every `try/except` block, every error return path, every health monitor check, and documented what WOULD happen if the condition occurred, even if I couldn't force it. For example, the UART buffer overflow entry says: "At 115200 baud, the hardware FIFO (128 bytes) fills in ~11ms. Our poll interval is 10ms. This is safe. If baud is increased to 921600 without reducing poll interval, overflow is guaranteed."

**Error 2: Error messages are inconsistent across subsystems.**
The logger's `error()` method logs as `ERROR`, but some modules used `warn()` for what I'd consider errors (like I2C failures). Others just printed to stdout with `print()`. This made the catalog confusing — the same underlying issue (sensor disconnected) could produce wildly different log output depending on which module detected it.

**Fix:** I standardised the error logging patterns. All sensor failures are now logged via `log.warn()` (rate-limited), and only after 50 consecutive failures does the log level escalate to `log.error()`. I documented this pattern in the catalog and added a note at the top: "All sensor errors follow the same pattern: warning (rate-limited) -> auto-disable -> error."

**Error 3: The catalog was too long to be useful at competition.**
My first draft was 15 pages. Nobody reads 15 pages of error messages during a 3-minute round. The judges have limited time too.

**Fix:** I added a one-page quick-reference table at the top, sorted by severity. CRITICAL errors are highlighted in red, HIGH in orange, etc. The detailed entries follow. I also added a "Competition Day" section with the 3 most likely errors and their 30-second fixes.

## Alternatives Considered

1. **Auto-generated error catalog from docstrings.** I could have written a script that parses every `log.error()` call and generates a markdown table. But it would miss the "why" and "how to handle" sections that require human judgement.

2. **Wiki-based error list.** A wiki would be more editable but isn't part of the repo. The judges expect to see this inside the repository.

3. **Separate ERROR_CODES.md for each subsystem.** I considered splitting by subsystem (sensors/errors.md, comms/errors.md, etc.) but a single file is easier to search on competition day.
