# v9.3 — README + Architecture

## What Changed

The repository root was a mess. There was a one-line README.md from initial setup, no architecture overview, and anyone cloning the repo would have no idea where to start. For the competition, the README is the judge's first impression — it needs to be polished, informative, and demonstrate engineering maturity.

I rewrote `README.md` completely and created `ARCHITECTURE.md` with a detailed data flow diagram.

**README.md changes:**
- Added CI badge showing build status
- Added shield badges for ESP-IDF version, Python version, license, WRO year
- Added 10-line ASCII architecture overview showing Pi -> ESP32 split
- Added "Key Engineering Highlights" section (12 bullet points showing design sophistication)
- Added full repository structure tree so judges can navigate immediately
- Added WRO Rule compliance table mapping rules 11.1-13.25 to our solutions
- Added Surprise Rule adaptation table showing all config changes
- Added Build & Run instructions for both Pi and ESP32
- Added Scoring table with all 122 points mapped
- Added Failure Analysis summary table
- Added Error Reference link

**ARCHITECTURE.md changes:**
- Full ASCII data flow diagram (80 chars wide, mobile-safe)
- Subsystem dependency graph
- Threading model with Hz rates for every task
- Key Design Decisions section (7 items explaining architectural choices)

## Errors Encountered and Fixed

**Error 1: ASCII diagrams look terrible on mobile GitHub.**
My first data flow diagram was a work of art — 120 characters wide, with fancy Unicode box-drawing characters (┌, ─, ┐, │, └, ┘, ├, ┤, ┴, ┬). On desktop it looked beautiful. When I checked on GitHub mobile, the lines were broken, boxes were misaligned, and text overflowed the viewport.

**Fix:** I redesigned every diagram to fit within 80 characters wide (the GitHub mobile default). I switched from Unicode box drawing to simple ASCII `+`, `-`, `|` characters. I also tested on both light and dark themes — some characters that looked fine on light were invisible on dark.

**Error 2: The README was too long.**
My first draft had 800 lines. It tried to document everything — every sensor, every algorithm, every design decision. A judge reading this would never finish.

**Fix:** I moved detailed descriptions to ARCHITECTURE.md and kept README.md as a quick-reference landing page. The rule is: README gets you started in 2 minutes, ARCHITECTURE.MD gives you the full picture in 10 minutes, docs/competition/ has the in-depth Appendix C documentation.

**Error 3: "This table cell wraps weirdly on narrow screens."**
The WRO Rule compliance table had a "Our Solution" column with long descriptions. On a narrow GitHub window, the table was unreadable — columns overlapped, text was truncated.

**Fix:** I shortened every cell description to under 40 characters and used hover-to-read detail strategy. The table now fits in a narrow viewport while still being informative.

## Alternatives Considered

1. **GitHub Pages site.** I considered creating a full GitHub Pages website with HTML docs. But static pages require maintenance and a separate build step. Judges will look at the repo files, not a website.

2. **Wiki-based README.** Some teams use GitHub Wikis for documentation. But wikis aren't part of the repository and won't be included in `git clone`.

3. **PDF documentation.** A nicely formatted PDF would look professional, but judges want to see the connection between docs and code. Hyperlinks in markdown provide that connection.

4. **Mermaid.js diagrams.** GitHub now supports Mermaid for diagrams in markdown. I tried it, but Mermaid renders server-side and some diagrams (especially data flows with many connections) looked cluttered. ASCII is simpler and always renders correctly.
