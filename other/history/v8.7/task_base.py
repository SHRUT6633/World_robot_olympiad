import asyncio
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class TaskContext:
    iteration: int = 0
    timestamp: float = 0.0
    dt_s: float = 0.0


class RobotTask(ABC):
    def __init__(self, name: str):
        self.name = name
        self._ctx = TaskContext()

    @abstractmethod
    async def execute(self, ctx: TaskContext):
        ...

    async def run(self):
        t0 = asyncio.get_event_loop().time()
        self._ctx.iteration += 1
        await self.execute(self._ctx)
        t1 = asyncio.get_event_loop().time()
        self._ctx.dt_s = t1 - t0
        self._ctx.timestamp = t1


class SensorTask(RobotTask):
    def __init__(self):
        super().__init__("sensors")
        self.accelerations: list[float] = []
        self.gyro: list[float] = []

    async def execute(self, ctx: TaskContext):
        pass


class PerceptionTask(RobotTask):
    def __init__(self):
        super().__init__("perception")
        self.detected_pillars: list[dict] = []

    async def execute(self, ctx: TaskContext):
        pass


class ControlTask(RobotTask):
    def __init__(self):
        super().__init__("control")
        self.steering_cmd: dict = {}

    async def execute(self, ctx: TaskContext):
        pass


class LoggingTask(RobotTask):
    def __init__(self):
        super().__init__("logging")
        self.buffer: list[str] = []

    async def execute(self, ctx: TaskContext):
        pass
