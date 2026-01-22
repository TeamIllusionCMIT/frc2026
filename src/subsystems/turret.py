from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax
from wpimath.controller import PIDController

from config import TurretConfig


class Turret(Subsystem):
    def __init__(self, config: TurretConfig):
        self.motor = SparkMax(config.port, SparkLowLevel.MotorType.kBrushless)
        self.controller = PIDController(config.pid.kP, config.pid.kI, config.pid.kD)
        self.controller.enableContinuousInput(-180, 180)

    def set_position(self, angle: float):
        """
        set the position of the turret. takes an angle in degrees.
        """
        self.controller.setSetpoint(angle)

    def get_position(self) -> float:
        return self.controller.getSetpoint()  # TODO: need to set up conversion

    def rotate(self, speed: float):
        self.motor.set(speed)

    def stop(self):
        self.motor.stopMotor()

    def periodic(self):
        self.motor.set(self.controller.calculate(self.get_position()))
