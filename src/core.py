from photonlibpy.targeting import PhotonTrackedTarget
from commands2 import ConditionalCommand
from commands2.button import CommandXboxController
from commands2.runcommand import RunCommand
from wpilib import ADIS16470_IMU, SmartDashboard, RobotController, DriverStation

from config import config
from src.subsystems.drivetrain import Drivetrain
from src.subsystems.odometry import Odometry
from src.subsystems.shooter import Shooter
from src.subsystems.intake import Intake
from src.subsystems.vision import Vision
from src.subsystems.turret import Turret
from math import sqrt


def deadband(value: float, threshold=0.02) -> float:
    """
    returns zero if the value is less than 0.02

    helps combat slight stick drift
    """
    return value if abs(value) > threshold else 0


def target_distance_from_camera_center(target: PhotonTrackedTarget) -> float:
    return sqrt(target.getYaw() ** 2 + target.getPitch() ** 2)


class RobotCore:
    """
    the core of the robot's functionality.
    """

    __slots__ = (
        "controller",
        "gyro",
        "drivetrain",
        "odometry",
        "vision",
        "shooter",
        "pose",
        "turret",
        "intake",
    )

    def __init__(self):
        self.controller = CommandXboxController(config.controller_port)
        self.gyro = ADIS16470_IMU()
        self.gyro.configCalTime(self.gyro.CalibrationTime(5))
        self.gyro.calibrate()
        # self.gyro.setSensitivity()
        self.gyro.reset()

        self.drivetrain = Drivetrain(config.motors, self.gyro)
        self.odometry = Odometry(self.gyro.getAngle())
        self.vision = Vision(config.vision.camera_name)
        self.shooter = Shooter(config.motors.shooter)
        self.turret = Turret(config.motors.turret)
        self.intake = Intake(config.motors.intake)

        self.pose = self.odometry.get_position()

        SmartDashboard.putData("pose", self.odometry.get_field())
        SmartDashboard.putData("gyro", self.gyro)
        self.configure_bindings()
        SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage())

    def turret_auto_aim(self):
        targets = self.vision.get_latest_targets()
        if not targets:
            return

        # sort targets by distance from camera center
        targets.sort(key=target_distance_from_camera_center)
        focused_target = targets[0]
        self.turret.aim_at_target(focused_target)

    def configure_bindings(self):
        # define drivetrain command.
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    deadband(self.controller.getLeftY()),
                    deadband(-self.controller.getLeftX()),
                    deadband(
                        -self.controller.getRightX()
                        if not self.controller.getLeftTriggerAxis() > 0.5
                        else 0
                    ),
                ),
                self.drivetrain,
            )
        )

        blue_targets = [18, 19, 20, 21, 24, 25, 26, 27]
        red_targets = [2, 3, 4, 5, 8, 9, 10, 11]
        alliance = DriverStation.getAlliance()
        target_ids: list[int] = []
        if alliance:
            target_ids.extend(
                red_targets if alliance == DriverStation.Alliance.kRed else blue_targets
            )
        else:
            target_ids.extend(blue_targets + red_targets)
        self.turret.setDefaultCommand(
            ConditionalCommand(
                RunCommand(
                    lambda: self.turret.rotate(self.controller.getRightX())
                    if self.controller.getRightX() > 0.1
                    else self.turret.set_position(0),
                    self.turret,
                ),
                RunCommand(
                    self.turret_auto_aim,
                    self.turret,
                ),
                lambda: self.controller.getHID().getRawButton(9),
            )
        )
        # self.controller.leftTrigger().whileTrue(
        #     RunCommand(
        #         lambda: self.turret.rotate(self.controller.getRightX()) if self.controller.getRightX() > 0.1 else self.turret.set_position(0),
        #         self.turret
        #     )
        # ).onFalse(RunCommand(self.turret.stop, self.turret))

        # set shooter and hood commands
        self.controller.leftBumper().onTrue(
            RunCommand(lambda: self.intake.shoot(-0.5), self.shooter),
        ).onFalse(RunCommand(self.shooter.stop, self.shooter))

        self.controller.rightBumper().onTrue(
            RunCommand(lambda: self.intake.shoot(0.5), self.shooter),
        ).onFalse(RunCommand(self.shooter.stop, self.shooter))

        self.controller.rightTrigger().onTrue(
            RunCommand(self.shooter.shoot, self.shooter)
        ).onFalse(RunCommand(self.shooter.stop, self.shooter))

    def periodic(self):
        wheel_positions = self.drivetrain.encoders.get_wheel_positions()
        self.pose = self.odometry.update_odometry(
            wheel_positions, self.gyro.getAngle(), None
        )
