"""Example of moving a motor using the supervisor."""

import argparse
import math
import time

from actuator import RobstrideMotors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/tty.usbserial-210")
    parser.add_argument("--motor-id", type=int, default=127)
    parser.add_argument("--motor-type", type=str, default="04")
    args = parser.parse_args()

    supervisor = RobstrideMotors(args.port_name, {args.motor_id: args.motor_type})
    supervisor.send_get_mode()
    start_time = time.time()

    try:
        while True:
            time.sleep(0.25)
            target_position = 0.5 * math.sin(time.time() - start_time)
            # supervisor.set_target_position(args.motor_id, target_position)
            feedback = supervisor.get_latest_feedback()
            print(feedback)

    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        raise


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
