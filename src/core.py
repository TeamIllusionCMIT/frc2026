from commands2.button import CommandXboxController
from commands2.runcommand import RunCommand
from wpilib import AnalogGyro

from config import config
from src.subsystems.vision import Vision
from subsystems.drivetrain import Drivetrain
from subsystems.odometry import Odometry


class RobotCore:
    """
    the core of the robot's functionality.
    """

    __slots__ = ("config", "controller", "gyro", "drivetrain", "odometry", "vision")

    def __init__(self):
        self.config = config
        self.controller = CommandXboxController(self.config.controller_port)
        self.gyro = AnalogGyro(self.config.gyro_port)
        self.gyro.reset()

        self.drivetrain = Drivetrain(self.config.motors, self.gyro)
        self.vision = Vision(self.config.vision.camera_name)
        self.odometry = Odometry(self.gyro.getAngle())

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

    def periodic(self):
        vision_estimate = self.vision.estimate_position()
        wheel_positions = self.drivetrain.encoders.get_wheel_positions()
        self.odometry.update_odometry(
            wheel_positions, self.gyro.getAngle(), vision_estimate
        )
