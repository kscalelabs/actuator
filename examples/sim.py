"""Example use of simulated actuators."""

import math
import time

from actuator import RobstrideMotorControlParams
from actuator.sim.robstride import RobstrideMotorsSim, RobstrideMotorsSupervisorSim


def test_robstride_motors_sim() -> None:
    # Define motor information
    motor_infos = {1: "01", 2: "02"}

    # Initialize the simulator
    motors_sim = RobstrideMotorsSim("virtual_port", motor_infos, verbose=True)

    # Send start command
    motors_sim.send_starts()

    # Define control parameters for the motors
    motor_controls = {
        1: RobstrideMotorControlParams(position=1.0, velocity=0.0, kp=10.0, kd=1.0, torque=0.0),
        2: RobstrideMotorControlParams(position=-1.0, velocity=0.0, kp=10.0, kd=1.0, torque=0.0),
    }

    # Send motor controls
    feedback = motors_sim.send_motor_controls(motor_controls, serial=False)
    print("Feedback:", feedback)

    # Send reset command
    motors_sim.send_resets()


def test_robstride_motors_supervisor_sim() -> None:
    # Define motor information
    motor_infos = {1: "01", 2: "02"}

    # Initialize the supervisor simulator
    supervisor_sim = RobstrideMotorsSupervisorSim(motor_infos, verbose=True)

    # Set a delay for the supervisor
    supervisor_sim.set_delay(0.01)

    # Define control parameters for the motors
    motor_controls = {
        1: RobstrideMotorControlParams(position=1.0, velocity=0.0, kp=10.0, kd=1.0, torque=0.0),
        2: RobstrideMotorControlParams(position=-1.0, velocity=0.0, kp=10.0, kd=1.0, torque=0.0),
    }

    start_time = time.time()

    try:
        while supervisor_sim.is_running():
            # Sinusoidal set point
            goal_pos = math.sin(2 * math.pi * (time.time() - start_time) / 5.0)
            motor_controls[1] = RobstrideMotorControlParams(
                position=goal_pos, velocity=0.0, kp=10.0, kd=1.0, torque=0.0
            )
            motor_controls[2] = RobstrideMotorControlParams(
                position=-goal_pos, velocity=0.0, kp=10.0, kd=1.0, torque=0.0
            )
            # Send motor controls
            feedback = supervisor_sim.send_motor_controls(motor_controls)
            print("Feedback:", feedback)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    # Stop the supervisor
    supervisor_sim.stop()


if __name__ == "__main__":
    print("Testing Robstride Motors Sim")
    test_robstride_motors_sim()
    print("\nTesting Robstride Motors Supervisor Sim")
    test_robstride_motors_supervisor_sim()
