from commands2 import CommandScheduler
from commands2.timedcommandrobot import TimedCommandRobot

from src.core import RobotCore


class Robot(TimedCommandRobot):
    __slots__ = ("core")
    def __init__(self):
        super().__init__()

        self.core = RobotCore()

    def robotPeriodic(self) -> None:
        self.core.periodic()
        CommandScheduler.getInstance().run()
