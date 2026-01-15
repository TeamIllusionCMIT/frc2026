from typing import NamedTuple

from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpimath.filter import SlewRateLimiter

from config import MotorConfig


class Encoders(NamedTuple):
    front_right_encoder: SparkRelativeEncoder
    front_left_encoder: SparkRelativeEncoder

    rear_right_encoder: SparkRelativeEncoder
    rear_left_encoder: SparkRelativeEncoder


class Drivetrain(Subsystem):
    def __init__(
        self, config: MotorConfig, motor_type=SparkLowLevel.MotorType.kBrushless
    ):
        super().__init__()

        self.front_right = SparkMax(config.front_right_port, motor_type)
        self.front_left = SparkMax(config.front_left_port, motor_type)

        self.rear_right = SparkMax(config.rear_right_port, motor_type)
        self.rear_left = SparkMax(config.rear_left_port, motor_type)

        self.encoders = Encoders(
            self.front_right.getEncoder(),
            self.front_left.getEncoder(),
            self.rear_right.getEncoder(),
            self.rear_left.getEncoder(),
        )

        self.drivetrain = MecanumDrive(
            self.front_left, self.rear_left, self.front_right, self.rear_right
        )
        self.drivetrain.setExpiration(0.1)

        self.forward_limiter = SlewRateLimiter(1)
        self.sideways_limiter = SlewRateLimiter(1)

    def drive(self, forward: float, sideways: float, rotate: float):
        self.drivetrain.driveCartesian(
            self.sideways_limiter.calculate(sideways),
            self.forward_limiter.calculate(forward),
            self.square_magnitude(rotate),
        )

    def square_magnitude(self, number: float):
        return (number**2) * (-1 if number < 0 else 1)
