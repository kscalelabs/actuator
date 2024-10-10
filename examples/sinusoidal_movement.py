"""Example of moving a motor in a sinusoidal pattern."""

import argparse
import logging
import math
import time

from actuator import RobstrideMotors

logger = logging.getLogger(__name__)


def run_motion_test(
    motors: RobstrideMotors,
    period: float = 1.0,
    motor_id: int = 1,
    max_torque: float = 10.0,
    amplitude: float = math.pi / 2.0,
    run_time_s: float = 10.0,
    kp: float = 1.0,
    kd: float = 0.01,
) -> None:
    motors.send_reset()
    motors.send_set_zero([motor_id])
    motors.send_start()

    start_time = time.time()
    command_count = 0

    try:
        while time.time() - start_time < run_time_s:
            dt = time.time() - start_time

            # Reference velocity is the derivative of the reference position.
            p_ref = amplitude * math.cos(dt * math.pi * 2.0 / period + math.pi / 2.0)
            v_ref = -amplitude * math.pi * 2.0 / period * math.sin(dt * math.pi * 2.0 / period + math.pi / 2.0)
            # v_ref = 0.0

            try:
                feedback = motors.get_latest_feedback_for(motor_id)
                p_cur = feedback.position
                v_cur = feedback.velocity
                torque = max(min(kp * (p_ref - p_cur) + kd * (v_ref - v_cur), max_torque), -max_torque)

                motors.send_torque_controls({motor_id: torque})

            except RuntimeError:
                logger.exception("Runtime error while getting latest feedback")
                motors.send_torque_controls({motor_id: 0.0})

            command_count += 1
            logger.info(
                "Motor %d Commands: %d, Frequency: %.2f Hz, Desired position: %.2f Feedback: %s",
                motor_id,
                command_count,
                command_count / dt,
                p_ref,
                feedback,
            )

    except KeyboardInterrupt:
        pass

    finally:
        motors.send_torque_controls({motor_id: 0.0})
        motors.send_reset()

    dt = time.time() - start_time
    logger.info(f"Done. Average control frequency: {command_count / dt:.2f} Hz")


def main() -> None:
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--max-torque", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=math.pi)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--run-time-s", type=float, default=10.0)
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=0.01)
    args = parser.parse_args()

    motors = RobstrideMotors(
        port_name=args.port_name,
        motor_infos={args.motor_id: "01"},
    )

    run_motion_test(
        motors,
        period=args.period,
        motor_id=args.motor_id,
        max_torque=args.max_torque,
        amplitude=args.amplitude,
        run_time_s=args.run_time_s,
        kp=args.kp,
        kd=args.kd,
    )


if __name__ == "__main__":
    # python -m examples.sinusoidal_movement
    main()
