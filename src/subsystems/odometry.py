from typing import Optional

from commands2 import Subsystem
from wpilib import AnalogGyro
from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

from constants import Chassis
from src.subsystems.drivetrain import Encoders


class Odometry(Subsystem):
    __slots__ = ("gyro", "kinematics", "pose_estimator")

    def __init__(self, gyro: AnalogGyro, encoders: Encoders):
        super().__init__()
        self.encoders = encoders
        self.gyro = gyro
        self.kinematics = MecanumDriveKinematics(
            frontRightWheel=Translation2d(
                Chassis.TRACK_WIDTH / 2, Chassis.WHEEL_BASE / 2
            ),
            frontLeftWheel=Translation2d(
                -Chassis.TRACK_WIDTH / 2, Chassis.WHEEL_BASE / 2
            ),
            rearRightWheel=Translation2d(
                Chassis.TRACK_WIDTH / 2, -Chassis.WHEEL_BASE / 2
            ),
            rearLeftWheel=Translation2d(
                -Chassis.TRACK_WIDTH / 2, -Chassis.WHEEL_BASE / 2
            ),
        )
        self.pose_estimator = MecanumDrivePoseEstimator(
            kinematics=self.kinematics,
            gyroAngle=Rotation2d(self.gyro.getAngle()),
            wheelPositions=MecanumDriveWheelPositions(),
            initialPose=Pose2d(),
        )

    def update_odometry(
        self,
        wheel_positions: MecanumDriveWheelPositions,
        vision_measurement: Optional[Pose2d] = None,
        vision_timestamp: Optional[float] = None,
    ) -> Pose2d:
        # TODO: custom vision measurement type that bundles these together
        if vision_measurement and vision_timestamp:
            self.pose_estimator.addVisionMeasurement(
                vision_measurement, vision_timestamp
            )
        return self.pose_estimator.update(
            Rotation2d(self.gyro.getAngle()), wheel_positions
        )

    def get_position(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def periodic(self) -> None:
        self.update_odometry(self.encoders.get_wheel_positions())
