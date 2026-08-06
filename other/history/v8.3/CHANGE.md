# v8.3 — Surprise Rule Configuration System

## What Changed

The WRO competition has a "surprise rule" element — at competition time, the judges announce additional rules that modify how the robot must behave. Instead of hardcoding these, I built a YAML-based configuration system that loads rule parameters at startup.

The module is `surprise_config.py`. It reads a `config.yaml` file containing fields like `pillar_logic`, `drive_direction`, `steering_mode`, `parking_mode`, and `penalties`. The system validates all fields against expected schemas and falls back to sensible defaults for missing fields.

The YAML structure looks like:
```yaml
surprise_rules:
  pillar_logic: pass_right
  drive_direction: forward
  steering_mode: same_phase
  parking_mode: parallel_reverse
  penalties:
    pillar_knock_over: 10
    line_cross: 5
    timeout: 20
```

## Errors Encountered

The first attempt to load the config on our Windows laptop failed with a cryptic error:

```
yaml.constructor.ConstructorError: unacceptable character #x001A
  in "config.yaml", line 1, column 1
```

I spent an hour debugging this. The `#x001A` character is a SUBSTITUTE character (Ctrl-Z), which is a Windows end-of-file marker. The file was saved with UTF-8 encoding with BOM, and some Windows editors append Ctrl-Z at the end. Python's YAML parser doesn't handle this gracefully.

Then the second error hit when we tested on the actual robot (Raspberry Pi running Linux):

```
yaml.scanner.ScannerError: mapping values are not allowed in this context
  in "config.yaml", line 3, column 16
```

This turned out to be a tab character in the YAML file. Our Windows editor inserted tabs instead of spaces, and the YAML spec requires spaces.

## The Fix

Two fixes were needed:

1. Force UTF-8 encoding when opening the file, and strip BOM characters:
```python
with open(path, 'r', encoding='utf-8-sig') as f:
    config = yaml.safe_load(f)
```

2. Add a YAML validation step that rejects tabs:
```python
def validate_yaml(content: str):
    for i, line in enumerate(content.split('\n'), 1):
        if '\t' in line:
            raise ValueError(f"Tab character found at line {i}")
```

The `utf-8-sig` codec handles the BOM automatically, and the tab check prevents the most common YAML formatting error.

## Alternatives Considered

1. **JSON instead of YAML**: JSON is simpler and less error-prone. But the surprise rules can include nested structures and comments (judges provide a printed rule sheet that we transcribe), and YAML is more readable for the non-programmers on the team who might need to edit the config at competition.

2. **Environment variables**: I could encode surprise rules as environment variables. This would avoid file parsing issues entirely. But we have 30+ config parameters and managing them as env vars would be unwieldy. Also, some parameters are nested structures (like penalty values per rule).

3. **Python config module**: Just edit a Python file with the config values. This would give us syntax validation for free. But the judges might restrict what code we can modify at competition (they sometimes lock the SD card), and a config file is less likely to be scrutinized.

4. **TOML format**: TOML is more Windows-friendly than YAML and doesn't have the tab/encoding issues. But Python's TOML support was only added in 3.11, and our robot runs 3.9. We'd need a third-party library, and the judges might not allow additional packages.

## Testing

- Loaded 10 different config files with various surprise rule combinations
- Tested UTF-8 BOM encoding: ✓ handled
- Tested UTF-16 encoding: ✓ rejected with clear error
- Tested tabs in YAML: ✓ rejected with line number
- Tested missing fields: ✓ defaults applied
- Tested all fields present: ✓ loaded correctly
- Tested on both Windows dev machine and Raspberry Pi: ✓ consistent behavior

## Lessons Learned

Cross-platform file encoding is still a pain in 2026. Always use `utf-8-sig` when reading text files on Windows. More importantly, validate early and validate loudly — a config parsing error at competition start is embarrassing but fixable; a silent fallback to wrong values could lose the match.
