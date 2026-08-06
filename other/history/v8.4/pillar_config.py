from dataclasses import dataclass


PILLAR_COLORS = ["red", "green", "blue", "yellow"]


@dataclass
class PillarSpec:
    color: str
    arUco_id: int
    expected_pass_side: str
    diameter_m: float = 0.1
    height_m: float = 0.3


PILLAR_DEFINITIONS = [
    PillarSpec(color="red", arUco_id=1, expected_pass_side="right"),
    PillarSpec(color="green", arUco_id=2, expected_pass_side="left"),
    PillarSpec(color="blue", arUco_id=3, expected_pass_side="left"),
    PillarSpec(color="yellow", arUco_id=4, expected_pass_side="right"),
]


def get_pillar_by_aruco_id(aruco_id: int) -> PillarSpec | None:
    for p in PILLAR_DEFINITIONS:
        if p.arUco_id == aruco_id:
            return p
    return None


def get_pillar_by_color(color: str) -> PillarSpec | None:
    for p in PILLAR_DEFINITIONS:
        if p.color == color:
            return p
    return None
