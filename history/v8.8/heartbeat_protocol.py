from dataclasses import dataclass


HEARTBEAT_TOPIC = "system/heartbeat"
EMERGENCY_STOP_TOPIC = "system/emergency_stop"


@dataclass
class HeartbeatMessage:
    task_name: str
    iteration: int
    timestamp: float
    cpu_usage_pct: float = 0.0
    memory_mb: float = 0.0


@dataclass
class EmergencyStopMessage:
    source: str
    reason: str
    timestamp: float


def format_heartbeat(msg: HeartbeatMessage) -> str:
    return (
        f"HB:{msg.task_name}:{msg.iteration}:"
        f"{msg.timestamp:.3f}:{msg.cpu_usage_pct:.1f}:{msg.memory_mb:.1f}"
    )


def parse_heartbeat(line: str) -> HeartbeatMessage | None:
    parts = line.strip().split(":")
    if len(parts) < 4 or parts[0] != "HB":
        return None
    return HeartbeatMessage(
        task_name=parts[1],
        iteration=int(parts[2]),
        timestamp=float(parts[3]),
        cpu_usage_pct=float(parts[4]) if len(parts) > 4 else 0.0,
        memory_mb=float(parts[5]) if len(parts) > 5 else 0.0,
    )
