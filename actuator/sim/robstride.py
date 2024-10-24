"""Simulated Robstride motors."""

import time

from actuator import RobstrideMotorControlParams, RobstrideMotorFeedback, RobstrideMotors, RobstrideMotorType

ROBSTRIDE_CONFIGS = RobstrideMotors.get_default_configs()


class _MotorSim:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.kp = 0.0
        self.kd = 0.0

    def update_state(self, params: RobstrideMotorControlParams) -> None:
        self.position = params.position
        self.velocity = params.velocity
        self.torque = params.torque
        self.kp = params.kp
        self.kd = params.kd

    def get_feedback(self) -> RobstrideMotorFeedback:
        feedback = RobstrideMotorFeedback.create_feedback(
            self.motor_id, self.position, self.velocity, self.torque, "Motor", 0
        )
        return feedback


class RobstrideMotorsSim:
    def __init__(self, port_name: str, motor_infos: dict[int, str], verbose: bool = False) -> None:
        self.port_name = port_name
        self.motor_configs = {
            id: ROBSTRIDE_CONFIGS[RobstrideMotorType.from_str(motor_type)] for id, motor_type in motor_infos.items()
        }
        self.verbose = verbose

        self.motors = {motor_id: _MotorSim(motor_id) for motor_id in motor_infos}  # Use MotorSim for each motor

    def send_get_mode(self) -> dict[int, str]:
        return {motor_id: "Motor" for motor_id in self.motors}

    def send_resets(self) -> None:
        for motor_id in self.motors:
            self.motors[motor_id] = _MotorSim(motor_id)

    def send_starts(self) -> None:
        if self.verbose:
            print("Motors started")

    def send_motor_controls(
        self,
        motor_controls: dict[int, RobstrideMotorControlParams],
        serial: bool,
    ) -> dict[int, RobstrideMotorFeedback]:
        feedback: dict[int, RobstrideMotorFeedback] = {}
        for motor_id, params in motor_controls.items():
            self.motors[motor_id].update_state(params)
            feedback[motor_id] = self.motors[motor_id].get_feedback()
        return feedback

    def __repr__(self) -> str:
        return f"RobstrideMotorsSim(port_name={self.port_name}, motor_configs={self.motor_configs})"


class RobstrideMotorsSupervisorSim:
    def __init__(self, motor_infos: dict[int, str], verbose: bool = False) -> None:
        self.verbose = verbose
        self.delay = 0.01  # Default delay in seconds

        self.target_params = {motor_id: RobstrideMotorControlParams(0, 0, 0, 0, 0) for motor_id in motor_infos}
        self.latest_feedback = {
            motor_id: RobstrideMotorFeedback.create_feedback(0, 0, 0, 0, "Reset", 0) for motor_id in motor_infos
        }

        self.running = True
        self.motors_sim = RobstrideMotorsSim("virtual_port", motor_infos, verbose)

    def set_delay(self, delay: float) -> None:
        """Set the delay for blocking communication methods."""
        self.delay = delay

    def send_motor_controls(
        self, motor_controls: dict[int, RobstrideMotorControlParams]
    ) -> dict[int, RobstrideMotorFeedback]:
        """Simulate sending motor controls with a blocking delay."""
        time.sleep(self.delay)  # Simulate blocking communication
        feedback = self.motors_sim.send_motor_controls(motor_controls, serial=False)
        self.latest_feedback.update(feedback)
        return self.latest_feedback.copy()

    def get_latest_feedback(self) -> dict[int, RobstrideMotorFeedback]:
        """Get the latest feedback from the motors."""
        return self.latest_feedback.copy()

    def stop(self) -> None:
        """Stop the supervisor."""
        self.running = False
        self.motors_sim.send_resets()

    def is_running(self) -> bool:
        """Check if the supervisor is running."""
        return self.running
