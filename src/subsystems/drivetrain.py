from typing import NamedTuple, Tuple

from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax, SparkRelativeEncoder
from wpilib import AnalogGyro
from wpilib.drive import MecanumDrive
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    MecanumDriveWheelPositions,
    MecanumDriveWheelSpeeds,
)

from config import MotorConfig
from constants import Chassis


class Encoders(NamedTuple):
    """
    a simple structure to hold the drivetrain encoders.

    members
    -------
    `get_wheel_positions()` gets the current wheel positions in rotations
    `get_wheel_speeds()` gets the current wheel speeds in RPM
    """

    front_right_encoder: SparkRelativeEncoder
    front_left_encoder: SparkRelativeEncoder

    rear_right_encoder: SparkRelativeEncoder
    rear_left_encoder: SparkRelativeEncoder

    def get_wheel_positions(self) -> MecanumDriveWheelPositions:
        """
        get current wheel positions (in rotations)
        """
        positions = MecanumDriveWheelPositions()

        positions.frontRight = self.front_right_encoder.getPosition()
        positions.frontLeft = self.front_left_encoder.getPosition()

        positions.rearRight = self.rear_right_encoder.getPosition()
        positions.rearLeft = self.rear_left_encoder.getPosition()

        return positions

    def get_wheel_speeds(self) -> MecanumDriveWheelSpeeds:
        """
        get current wheel speeds (in RPM)
        """
        return MecanumDriveWheelSpeeds(
            frontRight=self.front_right_encoder.getVelocity(),
            frontLeft=self.front_left_encoder.getVelocity(),
            rearRight=self.rear_right_encoder.getVelocity(),
            rearLeft=self.rear_left_encoder.getVelocity(),
        )


class Drivetrain(Subsystem):
    """
    the drivetrain subsystem.

    members
    -------
    `drive(forward, sideways, rotation)` to drive with controller inputs
    `drive_relative(speeds)` to drive with a `ChassisSpeeds` object
    """

    # using slots gives faster lookups and better memory efficiency.
    # great since it's going to be accessed ~50 times per second
    __slots__ = (
        "front_right",
        "front_left",
        "rear_right",
        "rear_left",
        "encoders",
        "drivetrain",
        "forward_limiter",
        "sideways_limiter",
        "gyro",
    )

    def __init__(
        self,
        config: MotorConfig,
        gyro: AnalogGyro,
        motor_type=SparkLowLevel.MotorType.kBrushless,
    ):
        super().__init__()

        self.gyro = gyro

        # initialize motors
        self.front_right = SparkMax(config.front_right_port, motor_type)
        self.front_left = SparkMax(config.front_left_port, motor_type)

        self.rear_right = SparkMax(config.rear_right_port, motor_type)
        self.rear_left = SparkMax(config.rear_left_port, motor_type)

        # create encoder object
        self.encoders = Encoders(
            self.front_right.getEncoder(),
            self.front_left.getEncoder(),
            self.rear_right.getEncoder(),
            self.rear_left.getEncoder(),
        )

        # initialize inner drivetrain
        self.drivetrain = MecanumDrive(
            self.front_left, self.rear_left, self.front_right, self.rear_right
        )
        self.drivetrain.setExpiration(0.1)

        # initialize slew rate limiters to soften acceleration
        self.forward_limiter = SlewRateLimiter(1)
        self.sideways_limiter = SlewRateLimiter(1)

    def drive(self, x_speed: float, y_speed: float, z_rotation: float):
        """
        drive the robot using controller inputs.

        automatically applies ratelimits to x/y motion and softens rotation.
        """

        if self.gyro:
            self.drivetrain.driveCartesian(
                xSpeed=self.sideways_limiter.calculate(x_speed),
                ySpeed=self.forward_limiter.calculate(y_speed),
                zRotation=self.square_magnitude(z_rotation),
                gyroAngle=self.gyro.getRotation2d(),
            )
        else:
            self.drivetrain.driveCartesian(
                xSpeed=self.sideways_limiter.calculate(x_speed),
                ySpeed=self.forward_limiter.calculate(y_speed),
                zRotation=self.square_magnitude(z_rotation),
            )

    @classmethod
    def square_magnitude(cls, number: float):
        """
        simple helper function to square a float without changing the sign, e.g. `square_magnitude(-4) == -16`
        """
        return (number**2) * (-1 if number < 0 else 1)

    @classmethod
    def normalize_chassis_speeds(
        cls, speeds: ChassisSpeeds
    ) -> Tuple[float, float, float]:
        """
        normalize chassis speeds to range 0.0-1.0
        """
        return (
            speeds.vx / Chassis.LINEAR_SPEED,
            speeds.vy / Chassis.LINEAR_SPEED,
            speeds.omega / Chassis.LINEAR_SPEED,
        )

    def drive_relative(self, speeds: ChassisSpeeds):
        """
        drives the robot using a pre-existing `ChassisSpeeds`.
        """
        self.drive(*self.normalize_chassis_speeds(speeds))
