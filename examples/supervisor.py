"""Example of moving a motor using the supervisor."""

import argparse
import logging
import time
import math

from actuator import RobstrideActuatorConfig, RobstrideSupervisor, RobstrideActuatorCommand, RobstrideConfigureRequest

def setup_logging() -> None:
    """Set up logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

def main() -> None:
    setup_logging()
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port-names",
        type=str,
        default="can0,can1",
        help="Comma-separated list of port names"
    )
    parser.add_argument("--motor-id", type=int, default=23)
    parser.add_argument("--motor-type", type=int, default=2)
    parser.add_argument("--polling-interval", type=float, default=0.001)
    parser.add_argument("--verbose", action="store_true")
    
    parser.add_argument("--test-sinusoidal", action="store_true", help="Enable sinusoidal motion testing")
    parser.add_argument("--sin-period", type=float, default=20.0, help="Period of sinusoidal motion in seconds")
    parser.add_argument("--sin-amplitude", type=float, default=10.0, help="Amplitude of sinusoidal motion in degrees")
    
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    ports = [port.strip() for port in args.port_names.split(",")]
    logger.info(f"Initializing supervisor with ports: {ports}")

    try:
        supervisor = RobstrideSupervisor(
            ports=ports,
            py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
            polling_interval=args.polling_interval,
        )
        logger.info("Supervisor initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize supervisor: {e}")
        return

    start_time = time.time()

    if args.test_sinusoidal:
        logger.info(f"Configuring motor {args.motor_id} for sinusoidal motion")
        supervisor.configure_actuator(RobstrideConfigureRequest(args.motor_id, kp=5.0, kd=1.0, max_torque=5.0, torque_enabled=True))

    try:
        while True:
            states = supervisor.get_actuators_state([args.motor_id])
            if states:
                for state in states:
                    status = "online" if state.online else "offline"
                    logger.info(f"Motor {state.actuator_id}: {status}")
                    if state.online:
                        logger.info(f"  Position: {state.position:.2f}°")
                        logger.info(f"  Velocity: {state.velocity:.2f}°/s")
                        logger.info(f"  Torque: {state.torque:.2f}")
                        logger.info(f"  Temperature: {state.temperature:.2f}°C")

            if args.test_sinusoidal:
                elapsed_time = time.time() - start_time
                target_position = args.sin_amplitude * math.sin(2 * math.pi * elapsed_time / args.sin_period)
                cmd = RobstrideActuatorCommand(args.motor_id, position=target_position_rad)
                supervisor.command_actuators([cmd])
                logger.info(f"  Target Position: {target_position:.2f}°")
            else:
                logger.warning("No motor states received")
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("\nShutting down gracefully...")


if __name__ == "__main__":
    main()
