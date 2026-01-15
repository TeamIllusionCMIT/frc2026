# frc 2026

this project uses [uv](https://github.com/astral-sh/uv) for package management.

to get started, run `uv sync`

additionally, we rely on the following packages:
- [msgspec](https://github.com/jcrist/msgspec) for configuration and constants 
- [pint](https://github.com/hgrecco/pint) for units and conversions

`robot.py` is fairly empty and only initializes the robot object. to change behavior, you're likely looking for `src/core.py`

subsystems for specific functionality is contained in `src/subsystems/`

to change configurable variables (like ports), see `config.toml`

to change mechanical constants (like motor specifications), see `constants.py`
