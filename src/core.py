from commands2.button import CommandXboxController
from commands2.runcommand import RunCommand
from wpilib import AnalogGyro

from config import config
from subsystems.drivetrain import Drivetrain
from subsystems.odometry import Odometry


class RobotCore:
    """
    the core of the robot's functionality.
    """

    __slots__ = ("config", "controller", "gyro", "drivetrain", "odometry")

    def __init__(self):
        self.config = config
        self.controller = CommandXboxController(self.config.controller_port)
        self.gyro = AnalogGyro(self.config.gyro_port)
        self.drivetrain = Drivetrain(self.config.motors, self.gyro)
        self.odometry = Odometry(self.drivetrain)

    def configure_bindings(self):
        # define drivetrain command.
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX(),
                ),
                self.drivetrain,
            )
        )
