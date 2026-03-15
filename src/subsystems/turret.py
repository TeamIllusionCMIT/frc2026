from photonlibpy.targeting import PhotonTrackedTarget
from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax

from config import PIDMotorConfig


class Turret(Subsystem):
    MIN_ANGLE_DEG = -90.0
    MAX_ANGLE_DEG = 90.0

    def __init__(self, config: PIDMotorConfig):
        self.motor = SparkMax(config.port, SparkLowLevel.MotorType.kBrushless)

        self.encoder = self.motor.getEncoder()
        self.encoder_offset = self.encoder.getPosition()

        self.controller = self.motor.getClosedLoopController()

    def set_position(self, angle: float):
        """
        set the position of the turret. takes an angle in degrees.
        """
        self.controller.setSetpoint(angle, SparkLowLevel.ControlType.kPosition)

    def get_position(self) -> float:
        return (
            self.encoder.getPosition() - self.encoder_offset
        )  # TODO: need to set up conversion

    def rotate(self, speed: float):
        self.motor.set(speed)

    def stop(self):
        self.motor.stopMotor()

    def aim_at_target(self, target: PhotonTrackedTarget):
        yaw_deg: int | float = target.getYaw()
        current_angle = self.get_position()

        # if the current moves the wrong way move this to current_angle - yaw_deg.
        desired_angle = max(
            self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, current_angle + yaw_deg)
        )
        self.set_position(desired_angle)

    # def periodic(self):
    #     self.motor.set(self.controller.calculate(self.get_position()))
