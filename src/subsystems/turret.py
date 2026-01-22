from commands2 import Subsystem
from rev import SparkMax, SparkLowLevel
from wpimath.controller import PIDController


class Turret(Subsystem):
    def __init__(self):
        self.motor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
        self.controller = PIDController(0, 0, 0)

    def set_position(self, angle: float):
        """
        set the position of the turret. takes an angle in degrees.
        """
        self.controller.setSetpoint(angle)

    def get_position(self) -> float:
        return self.controller.getSetpoint()  # TODO: need to set up conversion

    def periodic(self):
        self.motor.set(self.controller.calculate(self.get_position()))
