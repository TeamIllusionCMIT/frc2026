from typing import NamedTuple, Optional

from commands2 import Subsystem
from photonlibpy import EstimatedRobotPose, PhotonCamera, PhotonPoseEstimator
from photonlibpy.photonPoseEstimator import PhotonPipelineResult
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d

from constants import Chassis


class PhotonPoseEstimation(NamedTuple):
    pose: Pose2d
    timestamp: float

    @classmethod
    def from_estimation(cls, estimate: EstimatedRobotPose):
        return cls(
            pose=estimate.estimatedPose.toPose2d(),
            timestamp=estimate.timestampSeconds,
        )


class Vision(Subsystem):
    def __init__(self, camera_name: str):
        self.camera = PhotonCamera(camera_name)
        self.pose_estimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            Chassis.CAMERA_POSITION,
        )

    def estimate_position(self) -> Optional[PhotonPoseEstimation]:
        """
        estimate the current position based on the most recent photonvision result.
        """
        if not (result := self.get_latest_result()):
            return None

        estimation = self.pose_estimator.estimateCoprocMultiTagPose(result)

        return PhotonPoseEstimation.from_estimation(estimation) if estimation else None

    def get_latest_result(self) -> Optional[PhotonPipelineResult]:
        """
        gets the single latest result from the photonvision pipeline. can be `None`.
        """
        results = self.camera.getAllUnreadResults()
        return results[0] if len(results) > 0 else None
