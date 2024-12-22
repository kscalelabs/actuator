"""Example of moving a motor using the supervisor."""

import argparse
import logging
import time

from actuator import RobstrideActuatorConfig, RobstrideSupervisor, RobstrideActuatorCommand

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
            else:
                logger.warning("No motor states received")
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down gracefully...")


if __name__ == "__main__":
    main()
