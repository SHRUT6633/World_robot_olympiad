from dataclasses import dataclass


TRACK_WIDTH_M = 0.6
TRACK_LENGTH_M = 9.2


@dataclass
class SectionGeometry:
    name: str
    entry_x: float
    entry_y: float
    exit_x: float
    exit_y: float
    curvature_m: float
    length_m: float


SECTION_GEOMETRY = [
    SectionGeometry("start_straight", 0.0, 0.0, 1.5, 0.0, 0.0, 1.5),
    SectionGeometry("first_curve", 1.5, 0.0, 2.5, 1.0, 0.8, 1.5),
    SectionGeometry("mid_straight", 2.5, 1.0, 4.0, 1.0, 0.0, 1.5),
    SectionGeometry("pillar_zone", 4.0, 1.0, 5.5, 1.5, 0.3, 1.5),
    SectionGeometry("second_curve", 5.5, 1.5, 6.5, 0.5, 0.8, 1.5),
    SectionGeometry("parking_approach", 6.5, 0.5, 7.5, 0.2, 0.0, 1.0),
    SectionGeometry("parking_zone", 7.5, 0.2, 8.5, 0.0, 0.0, 0.7),
]


def get_section_behavior(section_name: str) -> str:
    behaviors = {
        "start_straight": "accelerate",
        "first_curve": "turn_left",
        "mid_straight": "cruise",
        "pillar_zone": "avoid_pillars",
        "second_curve": "turn_right",
        "parking_approach": "slow_down",
        "parking_zone": "parallel_park",
    }
    return behaviors.get(section_name, "unknown")


def is_straight(section_name: str) -> bool:
    for sec in SECTION_GEOMETRY:
        if sec.name == section_name:
            return sec.curvature_m == 0.0
    return True
