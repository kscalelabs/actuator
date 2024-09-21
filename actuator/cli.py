"""Defines the CLI for the actuator project."""

import time

from actuator.rust.lib import Actuator


def main() -> None:
    act = Actuator()
    time.sleep(3)
    print(act)


if __name__ == "__main__":
    # python -m actuator.cli
    main()
