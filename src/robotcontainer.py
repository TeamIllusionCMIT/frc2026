from commands2.button import CommandXboxController
from commands2.runcommand import RunCommand

from config import config
from subsystems.drivetrain import Drivetrain


class RobotContainer:
    def __init__(self):
        self.config = config
        self.controller = CommandXboxController(self.config.controller_port)
        self.drivetrain = Drivetrain(self.config.motors)

    def configure_bindings(self):
        # TODO: adjust the joystick ports
        self.controller.axisMagnitudeGreaterThan(0, 0.1).or_(
            self.controller.axisMagnitudeGreaterThan(1, 0.1)
        ).onTrue(
            RunCommand(
                lambda: self.drivetrain.drive(
                    self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX(),
                ),
                self.drivetrain,
            )
        )
