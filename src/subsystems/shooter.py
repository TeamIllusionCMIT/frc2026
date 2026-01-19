from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax
from wpimath.controller import PIDController


class Shooter(Subsystem):
    def __init__(self):
        self.hood_motor = SparkMax(6, SparkLowLevel.MotorType.kBrushless)
        self.hood_encoder = self.hood_motor.getEncoder()
        self.shooter = SparkMax(7, SparkLowLevel.MotorType.kBrushless)

        # this should become a ProfiledPIDController later
        # also these constants are entirely made up if you cant tell
        self.controller = PIDController(1, 1, 1)

    def periodic(self) -> None:
        measurement = self.hood_encoder.getPosition()
        self.hood_motor.set(self.controller.calculate(measurement))
