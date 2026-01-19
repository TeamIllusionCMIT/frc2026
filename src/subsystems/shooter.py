from commands2 import Subsystem
from rev import SparkLowLevel, SparkMax
from wpimath.controller import PIDController

from config import MotorConfig


class Shooter(Subsystem):
    __slots__ = ("hood_motor", "hood_encoder", "shooter", "auto", "controller")

    def __init__(self, config: MotorConfig):
        self.hood_motor = SparkMax(config.hood_port, SparkLowLevel.MotorType.kBrushless)
        self.hood_encoder = self.hood_motor.getEncoder()
        self.shooter = SparkMax(config.shooter_port, SparkLowLevel.MotorType.kBrushless)

        # this will be `True` when actively moving to a
        # setpoint and `False` otherwise
        self.auto = False

        # this should become a ProfiledPIDController later
        # also these constants are entirely made up if you cant tell
        pid_constants = config.hood_pid
        self.controller = PIDController(
            pid_constants.kP, pid_constants.kI, pid_constants.kD
        )

    def open(self) -> None:
        """
        open hood. makes shooter fire more upward and less forward. calling this will deactivate any setpoint.
        """
        self.auto = False
        self.hood_motor.set(1)

    def close(self) -> None:
        """
        close hood. makes shooter fire more forward and less upward. calling this will deactivate any setpoint.
        """
        self.auto = False
        self.hood_motor.set(-1)

    def stop(self) -> None:
        self.auto = False
        self.hood_motor.set(0)

    def shoot(self) -> None:
        """
        activate shooter
        """
        self.shooter.set(1)

    def stop_shooter(self) -> None:
        self.shooter.set(0)

    def set_setpoint(self, setpoint: float) -> None:
        self.auto = True
        self.controller.setSetpoint(setpoint)

    def get_setpoint(self) -> float:
        return self.controller.getSetpoint()

    def periodic(self) -> None:
        if self.auto:
            # if we're supposed to be running autonomously right now...

            if self.controller.atSetpoint():
                # and we've just hit the setpoint
                self.auto = False
            else:
                # otherwise, carry on
                measurement = self.hood_encoder.getPosition()
                self.hood_motor.set(self.controller.calculate(measurement))
