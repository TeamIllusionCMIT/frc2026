from wpilib import MotorControllerGroup
from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax
from wpimath.controller import PIDController

from config import PIDMotorConfig


class Intake(Subsystem):
    __slots__ = ("intake", "auto", "master")

    def __init__(self, config: PIDMotorConfig):
        self.intake = SparkMax(config.port, SparkLowLevel.MotorType.kBrushless)
        self.upper_intake = SparkMax(10, SparkLowLevel.MotorType.kBrushless)
        self.master = MotorControllerGroup(self.intake, self.upper_intake)

        # this will be `True` when actively moving to a
        # setpoint and `False` otherwise
        self.auto = False

        # this should become a ProfiledPIDController later
        self.bcontroller = self.intake.getClosedLoopController()
        # also these constants are entirely made up if you cant tell
        pid_constants = config.pid
        self.controller = PIDController(
            pid_constants.kP, pid_constants.kI, pid_constants.kD
        )

    def stop(self) -> None:
        self.master.set(0)

    def shoot(self, power: float = 0.5) -> None:
        """
        activate shooter
        """
        self.master.set(power)

    # def set_setpoint(self, setpoint: float) -> None:
    #     self.bcontroller.setSetpoint(
    #         setpoint, SparkLowLevel.ControlType.kVelocity
    #     )

    # def get_setpoint(self) -> float:
    #     return self.controller.getSetpoint()

    # def periodic(self) -> None:
    #     measurement = self.intake.getEncoder().getPosition()
    #     self.intake.set(self.controller.calculate(measurement))
