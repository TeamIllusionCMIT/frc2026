from commands2.button import CommandXboxController
from commands2.runcommand import RunCommand
from wpilib import AnalogGyro

from config import config
from src.subsystems.drivetrain import Drivetrain
from src.subsystems.odometry import Odometry
from src.subsystems.vision import Vision


def deadband(value: float, threshold=0.02) -> float:
    """
    returns zero if the value is less than 0.02

    helps combat slight stick drift
    """
    return value if abs(value) > threshold else 0


class RobotCore:
    """
    the core of the robot's functionality.
    """

    __slots__ = ("controller", "gyro", "drivetrain", "odometry", "vision", "pose")

    def __init__(self):
        self.controller = CommandXboxController(config.controller_port)
        self.gyro = AnalogGyro(config.gyro_port)
        self.gyro.reset()

        self.drivetrain = Drivetrain(config.motors, self.gyro)
        self.odometry = Odometry(self.gyro.getAngle())
        self.vision = Vision(config.vision.camera_name)

        self.pose = self.odometry.get_position()

        self.configure_bindings()

    def configure_bindings(self):
        # define drivetrain command.
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    deadband(self.controller.getLeftY()),
                    deadband(self.controller.getLeftX()),
                    deadband(self.controller.getRightX()),
                ),
                self.drivetrain,
            )
        )

    def periodic(self):
        vision_estimate = self.vision.estimate_position()
        wheel_positions = self.drivetrain.encoders.get_wheel_positions()
        self.pose = self.odometry.update_odometry(
            wheel_positions, self.gyro.getAngle(), vision_estimate
        )
