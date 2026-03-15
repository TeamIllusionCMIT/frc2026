# define objects that represent configured constants
# dataclasses keep this typed without an extra runtime dependency
import os
from dataclasses import dataclass
import tomllib
from typing import Any, Mapping
from pathlib import Path

# we're freezing dataclasses to improve performance
# at the cost of mutability (the ability to edit)
# which we don't need anyway


@dataclass(frozen=True)
class PIDConfig:
    kP: float
    kI: float
    kD: float


@dataclass(frozen=True)
class PIDMotorConfig:
    port: int
    pid: PIDConfig


@dataclass(frozen=True)
class MotorConfig:
    front_right_port: int
    front_left_port: int

    rear_right_port: int
    rear_left_port: int

    shooter: PIDMotorConfig

    turret: PIDMotorConfig
    intake: PIDMotorConfig


@dataclass(frozen=True)
class PhotonVisionConfig:
    camera_name: str


@dataclass(frozen=True)
class ConfigFile:
    controller_port: int
    gyro_port: int

    motors: MotorConfig
    vision: PhotonVisionConfig


def _build_pid_config(data: Mapping[str, Any]) -> PIDConfig:
    return PIDConfig(kP=data["P"], kI=data["I"], kD=data["D"])


def _build_pid_motor_config(data: Mapping[str, Any]) -> PIDMotorConfig:
    return PIDMotorConfig(port=data["port"], pid=_build_pid_config(data["pid"]))


# be aware of the current working directory
script_path = Path(__file__).resolve().parent
with open(script_path.joinpath("config.toml"), "rb") as config_file:
    raw_config = tomllib.load(config_file)

motors = raw_config["motors"]
config = ConfigFile(
    controller_port=raw_config["controller_port"],
    gyro_port=raw_config["gyro_port"],
    motors=MotorConfig(
        front_right_port=motors["front_right_port"],
        front_left_port=motors["front_left_port"],
        rear_right_port=motors["rear_right_port"],
        rear_left_port=motors["rear_left_port"],
        shooter=_build_pid_motor_config(motors["shooter"]),
        turret=_build_pid_motor_config(motors["turret"]),
        intake=_build_pid_motor_config(motors["intake"]),
    ),
    vision=PhotonVisionConfig(camera_name=raw_config["vision"]["camera_name"]),
)
