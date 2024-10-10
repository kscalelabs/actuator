"""Example of moving a motor using the supervisor with WASD control."""

import argparse
import time
import curses

from actuator import RobstrideMotorsSupervisor

def main(stdscr) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyCH341USB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=str, default="04")
    parser.add_argument("--second-motor-id", type=int, default=2)  # New argument for second motor ID
    parser.add_argument("--second-motor-type", type=str, default="01")  # New argument for second motor type
    args = parser.parse_args()

    motor_infos = {
        args.motor_id: args.motor_type,
        args.second_motor_id: args.second_motor_type  # Use new arguments
    }
    supervisor = RobstrideMotorsSupervisor(args.port_name, motor_infos)
    supervisor.add_motor_to_zero(args.motor_id)
    supervisor.add_motor_to_zero(args.second_motor_id)  # Use new argument

    position_motor_1 = 0.0  # Initial position for motor 1
    position_motor_2 = 0.0  # Initial position for motor 2
    normal_step_size = 0.5  # Normal step size for position change
    high_step_size = 1   # High step size for position change
    low_step_size = 0.25  # Low step size for position change

    stdscr.nodelay(True)  # Make getch non-blocking
    stdscr.clear()

    try:
        while True:
            time.sleep(0.01)

            # Check for key presses
            key = stdscr.getch()

            if key == ord('a'):
                position_motor_1 -= normal_step_size  # Move motor 1 counter-clockwise
            elif key == ord('d'):
                position_motor_1 += normal_step_size  # Move motor 1 clockwise
            elif key == ord('w'):
                position_motor_2 += normal_step_size  # Move motor 2 clockwise
            elif key == ord('s'):
                position_motor_2 -= normal_step_size  # Move motor 2 counter-clockwise

            # Set target position for both motors
            supervisor.set_target_position(args.motor_id, position_motor_1)
            supervisor.set_target_position(args.second_motor_id, position_motor_2)

            feedback = supervisor.get_latest_feedback()
            stdscr.addstr(0, 0, f"Motor 1 Position: {position_motor_1:.2f}, Motor 2 Position: {position_motor_2:.2f}, Feedback: {feedback}")
            stdscr.refresh()

    except KeyboardInterrupt:
        supervisor.stop()
        time.sleep(0.1)
        raise

if __name__ == "__main__":
    # python -m examples.supervisor
    curses.wrapper(main)
