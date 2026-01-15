from math import pi, sqrt

from msgspec import Struct
from pint import UnitRegistry
from wpimath.geometry import Transform3d, Translation2d
from wpimath.kinematics import MecanumDriveKinematics

units = UnitRegistry()


class VortexMotorConstants(Struct, frozen=True):
    FREE_SPEED = 5676 * units.rpm
    ENCODER_RESOLUTION = (
        42 * units.counts / units.revolution
    )  # in counts per revolution


class AprilTagConstants(Struct, frozen=True):
    # this is necessary to make it not angry
    APRILTAG_WIDTH = units.Quantity(8.125 * units.inch).to(units.meter)

    # TODO: add other stuff like locations


class Chassis(Struct, frozen=True):
    # body
    LENGTH = units.Quantity(0.82 * units.meter)
    WIDTH = units.Quantity(0.67 * units.meter)

    # wheel and track
    WHEEL_RADIUS = units.Quantity(5 * units.inch).to(units.meter)
    WHEEL_DIAMETER = WHEEL_RADIUS * 2  # in meters
    WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi * 2

    ## distance between left and right wheels, in meters
    TRACK_WIDTH = units.Quantity(5 * units.inch).to(units.meter)
    ## distance from front wheels to back wheels, in meters
    WHEEL_BASE = units.Quantity(5 * units.inch).to(units.meter)
    ## gear ratio of the drivetrain. from the kitbot datasheet
    GEAR_RATIO: float = 8.450

    # derived from other things
    LINEAR_SPEED = ((WHEEL_RADIUS * VortexMotorConstants.FREE_SPEED) / GEAR_RATIO).to(
        units.mps
    )  # in m/s
    ROBOT_RADIUS = (
        sqrt(((LENGTH.magnitude / 2) ** 2) + ((WIDTH.magnitude / 2) ** 2))
    ) * units.meter  # in meters
    ANGULAR_SPEED = (LINEAR_SPEED / ROBOT_RADIUS).to(units("rad/s"))  # in rad/s

    # TODO: actually set this
    CAMERA_POSITION = Transform3d()

    KINEMATICS = MecanumDriveKinematics(
        frontRightWheel=Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2),
        frontLeftWheel=Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2),
        rearRightWheel=Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2),
        rearLeftWheel=Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2),
    )
