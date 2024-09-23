"""Runs a simple test using PythonCAN."""

import argparse
import asyncio
import logging
import math
import struct
from enum import IntEnum

import can

logger = logging.getLogger(__name__)


class StopMode(IntEnum):
    Normal = 0
    ClearErr = 1


class RobStrideUtils:
    def __init__(self, node_id: int) -> None:
        self.id: int = node_id

    def set_extended_id(self, com_type: int, data_field: int, destination: int) -> int:
        return (com_type << 24) | (data_field << 8) | destination

    def request_dev_id(self) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(0, 0x0000, self.id),
            data=[0],
            is_extended_id=True,
        )

    def request_motion(self, torque: float, angle: float, velocity: float, kp: float, kd: float) -> can.Message:
        torqueu16 = int((torque + 17 / 34) * 65535)
        angleu16 = int((angle + 4 * math.pi / (8 * math.pi)) * 65535)
        velocityu16 = int((velocity + 44 / 88) * 65535)
        kpu16 = int((kp / 500) * 65535)
        kdu16 = int((kd / 5) * 65535)

        return can.Message(
            arbitration_id=self.set_extended_id(1, torqueu16, self.id),
            data=[
                (angleu16 >> 8) & 0xFF,
                angleu16 & 0xFF,
                (velocityu16 >> 8) & 0xFF,
                velocityu16 & 0xFF,
                (kpu16 >> 8) & 0xFF,
                kpu16 & 0xFF,
                (kdu16 >> 8) & 0xFF,
                kdu16 & 0xFF,
            ],
            is_extended_id=True,
        )

    def request_enable(self) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(3, 0x0000, self.id),
            data=[0],
            is_extended_id=True,
        )

    def request_stop(self, mode: StopMode) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(4, 0x0000, self.id),
            data=[mode],
            is_extended_id=True,
        )

    def set_zero_point(self) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(6, 0x0000, self.id),
            data=[1],
            is_extended_id=True,
        )

    def set_dev_id(self, request_id: int) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(7, (request_id << 8), self.id),
            data=[1],
            is_extended_id=True,
        )

    def request_param(self, index: int) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(17, 0x0000, self.id),
            data=[
                index & 0xFF,
                (index >> 8) & 0xFF,
            ],
            is_extended_id=True,
        )

    def write_param(self, index: int, param: int) -> can.Message:
        return can.Message(
            arbitration_id=self.set_extended_id(18, 0x0000, self.id),
            data=[
                index & 0xFF,
                (index >> 8) & 0xFF,
                0,
                0,
                param & 0xFF,
                (param >> 8) & 0xFF,
                (param >> 16) & 0xFF,
                (param >> 24) & 0xFF,
            ],
            is_extended_id=True,
        )


async def main() -> None:
    logging.basicConfig(level=logging.DEBUG)

    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("tty_device", type=str, help="The tty device to use")
    parser.add_argument("-b", "--bitrate", type=int, default=500000, help="The bitrate to use")
    parser.add_argument("-i", "--interface", type=str, default="slcan", help="The interface to use")
    args: argparse.Namespace = parser.parse_args()

    bus: can.BusABC = can.interface.Bus(bustype=args.interface, channel=args.tty_device, bitrate=args.bitrate)
    rs_util: RobStrideUtils = RobStrideUtils(0x7F)

    # Send messages
    bus.send(rs_util.request_dev_id())
    bus.send(rs_util.request_enable())
    bus.send(rs_util.request_motion(0.0, 1.0, 0.0, 0.2, 0.1))

    # Spawn a task to send a parameter request
    async def send_param_request() -> None:
        await asyncio.sleep(0.001)  # 1ms delay
        bus.send(rs_util.request_param(0x302D))

    asyncio.create_task(send_param_request())

    # Receive messages
    try:
        while True:
            message: can.Message | None = bus.recv(1)  # 1 second timeout
            if message is None:
                continue

            if message.is_error_frame:
                logger.error("Error frame: %s", message)
            elif message.is_remote_frame:
                logger.info("Remote frame: %s", message)
            else:
                logger.info("Data frame: %s", message)
                if len(message.data) == 8:
                    # Assuming the last 4 bytes contain a float value
                    float_value: float = struct.unpack("<f", message.data[4:8])[0]
                    logger.info("Float value: %s", float_value)

    except KeyboardInterrupt:
        logger.error("Interrupted by user")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
