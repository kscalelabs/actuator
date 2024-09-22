"""Defines the CLI for the actuator project."""

import time

from actuator.rust.lib import Actuator, ActuatorType


def main() -> None:
    act_1 = Actuator(ActuatorType.Robstride, True)
    act_1.set_position(0, 0.5)
    act_2 = Actuator(ActuatorType.Robstride, True)
    act_2.set_position(1, 0.5)
    act_2.set_pid_params(1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    time.sleep(5)
    print(act_1)
    print(act_2)


if __name__ == "__main__":
    # python -m actuator.cli
    main()
