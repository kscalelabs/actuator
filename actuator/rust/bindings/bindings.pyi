# This file is automatically generated by pyo3_stub_gen
# ruff: noqa: E501, F401

import typing

class PyRobstrideMotorFeedback:
    can_id: int
    position: float
    velocity: float
    torque: float
    mode: str
    faults: int
    def __repr__(self) -> str:
        ...


class PyRobstrideMotors:
    def __new__(cls,port_name:str, motor_infos:typing.Mapping[int, str]): ...
    def send_get_mode(self) -> dict[int, str]:
        ...

    def send_set_zero(self, motor_ids:typing.Optional[typing.Sequence[int]]) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_reset(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_start(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_torque_controls(self, torque_sets:typing.Mapping[int, float]) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def get_latest_feedback(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def get_latest_feedback_for(self, motor_id:int) -> PyRobstrideMotorFeedback:
        ...

    def __repr__(self) -> str:
        ...


class PyRobstrideMotorsSupervisor:
    def __new__(cls,port_name:str, motor_infos:typing.Mapping[int, str]): ...
    def set_target_position(self, motor_id:int, position:float) -> None:
        ...

    def set_kp_kd(self, motor_id:int, kp:float, kd:float) -> None:
        ...

    def set_sleep_duration(self, sleep_duration:float) -> None:
        ...

    def add_motor_to_zero(self, motor_id:int) -> None:
        ...

    def get_latest_feedback(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def stop(self) -> None:
        ...

    def __repr__(self) -> str:
        ...


