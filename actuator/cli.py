"""Defines the CLI for the actuator project."""

import math
import time

from actuator.rust.py import PyRobstrideMotors

# Constants (you may need to adjust these based on your Python implementation)
RUN_TIME = 3.0  # Assuming 10 seconds, adjust as needed
TEST_ID = 2  # Assuming motor ID 1, adjust as needed
MAX_TORQUE = 1.0  # Adjust based on your motor specifications
PI = math.pi


def run_motion_test(motors: PyRobstrideMotors) -> None:
    motors.send_reset()
    motors.send_set_zero()
    motors.send_start()

    start_time = time.time()
    command_count = 0

    # PD controller parameters
    kp_04 = 0.5
    kd_04 = 0.1

    # Define period and amplitude
    period = RUN_TIME
    amplitude = PI / 1.0

    while time.time() - start_time < RUN_TIME:
        elapsed_time = time.time() - start_time
        desired_position = amplitude * math.cos(elapsed_time * PI * 2.0 / period + PI / 2.0)

        feedback = motors.get_latest_feedback()[TEST_ID]
        current_position = feedback.position
        current_velocity = feedback.velocity
        torque = max(
            min(kp_04 * (desired_position - current_position) - kd_04 * current_velocity, MAX_TORQUE), -MAX_TORQUE
        )

        motors.send_torque_controls({TEST_ID: torque})

        command_count += 1
        print(
            f"Motor {TEST_ID} Commands: {command_count}, Frequency: {command_count / elapsed_time:.2f} Hz, "
            f"Desired position: {desired_position:.2f} Feedback: {feedback}"
        )

    motors.send_torque_controls({TEST_ID: 0.0})
    motors.send_reset()

    elapsed_time = time.time() - start_time
    print(f"Done. Average control frequency: {command_count / elapsed_time:.2f} Hz")


def main() -> None:
    motors = PyRobstrideMotors(
        port_name="/dev/ttyUSB0",
        motor_infos={TEST_ID: "01"},
    )
    run_motion_test(motors)


if __name__ == "__main__":
    # python -m actuator.cli
    main()
