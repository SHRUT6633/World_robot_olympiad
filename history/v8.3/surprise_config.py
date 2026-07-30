import yaml
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Penalties:
    pillar_knock_over: int = 10
    line_cross: int = 5
    timeout: int = 20
    parking_misalignment_mm: int = 2


@dataclass
class SurpriseRules:
    pillar_logic: str = "pass_right"
    drive_direction: str = "forward"
    steering_mode: str = "same_phase"
    parking_mode: str = "parallel_reverse"
    penalties: Penalties = field(default_factory=Penalties)


PILLAR_LOGIC_OPTIONS = {"pass_left", "pass_right", "alternate"}
DRIVE_DIRECTION_OPTIONS = {"forward", "reverse", "bidirectional"}
STEERING_MODE_OPTIONS = {"same_phase", "opposite_phase", "crab_walk"}
PARKING_MODE_OPTIONS = {"parallel_reverse", "parallel_forward", "perpendicular"}


def validate_yaml(content: str) -> None:
    for i, line in enumerate(content.split("\n"), 1):
        if "\t" in line:
            raise ValueError(
                f"Tab character found at line {i}. YAML requires spaces."
            )


def check_field(value: Any, options: set[str], field_name: str) -> None:
    if value not in options:
        raise ValueError(
            f"Invalid {field_name}: '{value}'. Must be one of {options}"
        )


def load_config(path: str | Path) -> SurpriseRules:
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    raw = path.read_text(encoding="utf-8-sig")
    validate_yaml(raw)

    data = yaml.safe_load(raw)
    if data is None:
        return SurpriseRules()

    rules_data = data.get("surprise_rules", {})
    rules = SurpriseRules()

    if "pillar_logic" in rules_data:
        check_field(rules_data["pillar_logic"], PILLAR_LOGIC_OPTIONS, "pillar_logic")
        rules.pillar_logic = rules_data["pillar_logic"]

    if "drive_direction" in rules_data:
        check_field(rules_data["drive_direction"], DRIVE_DIRECTION_OPTIONS, "drive_direction")
        rules.drive_direction = rules_data["drive_direction"]

    if "steering_mode" in rules_data:
        check_field(rules_data["steering_mode"], STEERING_MODE_OPTIONS, "steering_mode")
        rules.steering_mode = rules_data["steering_mode"]

    if "parking_mode" in rules_data:
        check_field(rules_data["parking_mode"], PARKING_MODE_OPTIONS, "parking_mode")
        rules.parking_mode = rules_data["parking_mode"]

    if "penalties" in rules_data:
        p = rules_data["penalties"]
        if "pillar_knock_over" in p:
            rules.penalties.pillar_knock_over = int(p["pillar_knock_over"])
        if "line_cross" in p:
            rules.penalties.line_cross = int(p["line_cross"])
        if "timeout" in p:
            rules.penalties.timeout = int(p["timeout"])

    return rules
