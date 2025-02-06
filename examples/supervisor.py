"""Example of moving a motor using the supervisor."""

import argparse
import time

from actuator import RobstrideActuator, RobstrideActuatorConfig
from dataclasses import dataclass
import threading
import logging

logging.getLogger().setLevel(logging.INFO)


@dataclass
class ActuatorState:
    actuator_id: int
    online: bool
    position: float
    velocity: float
    torque: float
    temperature: float


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyCH341USB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=int, default=4)
    parser.add_argument("--sleep", type=float, default=0.1)
    parser.add_argument("--period", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--update_loop_interval_ms", type=int, default=10)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    _amplitude = args.amplitude
    _period = args.period
    supervisor = RobstrideActuator(
        ports=[args.port_name], py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))]
    )

    supervisor.disable(args.motor_id)
    target_fps = 30
    frame_time = 1.0 / target_fps
    start_time = time.time()
    supervisor.run_main_loop(interval_ms=args.update_loop_interval_ms)

    def monitor_thread():
        nonlocal start_time
        while True:
            loop_start = time.time()

            state = supervisor.get_actuators_state([args.motor_id])
            if state:
                pos = state[0].position
                print("position: ", pos)

            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = 1.0 / ((time.time() - start_time) / target_fps)
                print("Actual Fps", fps)
                start_time = time.time()

            sleep_time = frame_time - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    monitor = threading.Thread(target=monitor_thread)
    control = threading.Thread(target=control_thread)
    try:
        monitor.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
