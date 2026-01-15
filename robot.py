from commands2 import CommandScheduler
from commands2.timedcommandrobot import TimedCommandRobot

from src.robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    def __init__(self):
        super().__init__()

        self.core = RobotContainer()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
