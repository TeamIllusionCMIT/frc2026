from wpilib import Field2d
from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import MecanumDriveWheelPositions

from constants import Chassis
from src.subsystems.vision import PhotonPoseEstimation


class Odometry:
    __slots__ = ("pose_estimator", "field")

    def __init__(self, starting_angle: float = 0):
        self.pose_estimator = MecanumDrivePoseEstimator(
            kinematics=Chassis.KINEMATICS,
            gyroAngle=Rotation2d(starting_angle),
            wheelPositions=MecanumDriveWheelPositions(),
            initialPose=Pose2d(),
        )
        self.field = Field2d()

    def update_odometry(
        self,
        wheel_positions: MecanumDriveWheelPositions,
        angle: float,
        vision_estimate: PhotonPoseEstimation | None,
    ) -> Pose2d:
        # TODO: custom vision measurement type that bundles these together
        if vision_estimate:
            self.pose_estimator.addVisionMeasurement(
                vision_estimate.pose, vision_estimate.timestamp
            )
        result = self.pose_estimator.update(Rotation2d(angle), wheel_positions)
        self.field.setRobotPose(result)
        return result

    def get_position(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def get_field(self) -> Field2d:
        """
        get the robot's current position as a Field2d
        """
        return self.field
