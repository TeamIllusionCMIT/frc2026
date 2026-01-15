# define obejcts that will represent configured constants
# it uses msgspec so that we can have everything be typed
from msgspec import Struct
from msgspec.toml import decode as decode_toml


# we're freezing structs to improve performance
# at the cost of mutability (the ability to edit)
# which we don't need anyway
class MotorConfig(Struct, frozen=True):
    front_right_port: int
    front_left_port: int

    rear_right_port: int
    rear_left_port: int


class PhotonVisionConfig(Struct, frozen=True):
    pass


class ConfigFile(Struct):
    controller_port: int
    motors: MotorConfig
    vision: PhotonVisionConfig


config_file = open("config.toml")
config = decode_toml(config_file.read(), type=ConfigFile)
config_file.close()
