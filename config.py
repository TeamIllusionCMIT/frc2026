# define obejcts that will represent configured constants
# it uses msgspec so that we can have everything be typed
from msgspec import Struct, field
from msgspec.toml import decode as decode_toml

# we're freezing structs to improve performance
# at the cost of mutability (the ability to edit)
# which we don't need anyway


class PIDConfig(Struct, frozen=True):
    kP: float = field(name="P")
    kI: float = field(name="I")
    kD: float = field(name="D")


class TurretConfig(Struct, frozen=True):
    port: int
    pid: PIDConfig


class MotorConfig(Struct, frozen=True):
    front_right_port: int
    front_left_port: int

    rear_right_port: int
    rear_left_port: int

    hood_port: int
    shooter_port: int

    hood_pid: PIDConfig
    turret: TurretConfig


class PhotonVisionConfig(Struct, frozen=True):
    camera_name: str


class ConfigFile(Struct):
    controller_port: int
    gyro_port: int

    motors: MotorConfig
    vision: PhotonVisionConfig


config_file = open("config.toml")
config = decode_toml(config_file.read(), type=ConfigFile)
config_file.close()
