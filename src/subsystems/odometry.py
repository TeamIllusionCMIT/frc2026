from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

from constants import Chassis
from src.subsystems.vision import PhotonPoseEstimation


class Odometry:
    __slots__ = "pose_estimator"

    def __init__(self, starting_angle: float = 0):
        self.pose_estimator = MecanumDrivePoseEstimator(
            kinematics=Chassis.KINEMATICS,
            gyroAngle=Rotation2d(starting_angle),
            wheelPositions=MecanumDriveWheelPositions(),
            initialPose=Pose2d(),
        )

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
        return self.pose_estimator.update(Rotation2d(angle), wheel_positions)

    def get_position(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()
