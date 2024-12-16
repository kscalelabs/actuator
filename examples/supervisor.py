"""Example of moving a motor using the supervisor."""

import argparse
import math
import time

from actuator import RobstrideActuator, RobstrideActuatorConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyCH341USB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="04")
    parser.add_argument("--sleep", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    _amplitude = args.amplitude
    _period = args.period

    supervisor = RobstrideActuator(
        ports=[args.port_name],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
        polling_interval=args.sleep,
    )

    supervisor.start()

if __name__ == "__main__":
    # python -m examples.supervisor
    main()
