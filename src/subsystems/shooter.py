from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax
from wpimath.controller import PIDController

from config import PIDMotorConfig


class Shooter(Subsystem):
    __slots__ = ("shooter", "auto")

    def __init__(self, config: PIDMotorConfig):
        self.shooter = SparkMax(config.port, SparkLowLevel.MotorType.kBrushless)

        # this will be `True` when actively moving to a
        # setpoint and `False` otherwise
        self.auto = False

        # this should become a ProfiledPIDController later
        # also these constants are entirely made up if you cant tell
        pid_constants = config.pid
        self.controller = PIDController(
            pid_constants.kP, pid_constants.kI, pid_constants.kD
        )

    def stop(self) -> None:
        self.shooter.set(0)

    def shoot(self, power: float = 1.0) -> None:
        """
        activate shooter
        """
        self.shooter.set(power)

    def set_setpoint(self, setpoint: float) -> None:
        self.controller.setSetpoint(setpoint)

    def get_setpoint(self) -> float:
        return self.controller.getSetpoint()

    def periodic(self) -> None:
        # measurement = self.shooter.getEncoder().getPosition()
        # self.shooter.set(self.controller.calculate(measurement))
        ...
