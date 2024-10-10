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

class PyRobstrideMotors:
    def __new__(cls,port_name:str, motor_infos:typing.Sequence[tuple[int, str]]): ...
    def send_get_mode(self) -> dict[int, str]:
        ...

    def send_set_zero(self) -> dict[int, PyRobstrideMotorFeedback]:
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


