# This file is automatically generated by pyo3_stub_gen
# ruff: noqa: E501, F401

import typing

class PyRobstrideMotorControlParams:
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float
    def __new__(cls,position:float, velocity:float, kp:float, kd:float, torque:float): ...
    def __repr__(self) -> str:
        ...


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
    def __new__(cls,port_name,motor_infos,verbose = ...): ...
    def send_get_mode(self) -> dict[int, str]:
        ...

    def send_set_zero(self, motor_ids:typing.Optional[typing.Sequence[int]]) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_reset(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_start(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def send_motor_controls(self, motor_controls:typing.Mapping[int, PyRobstrideMotorControlParams]) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def get_latest_feedback(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def get_latest_feedback_for(self, motor_id:int) -> PyRobstrideMotorFeedback:
        ...

    def __repr__(self) -> str:
        ...


class PyRobstrideMotorsSupervisor:
    def __new__(cls,port_name:str, motor_infos:typing.Mapping[int, str], verbose:bool): ...
    def set_position(self, motor_id:int, position:float) -> None:
        ...

    def set_velocity(self, motor_id:int, velocity:float) -> None:
        ...

    def set_kp(self, motor_id:int, kp:float) -> None:
        ...

    def set_kd(self, motor_id:int, kd:float) -> None:
        ...

    def set_torque(self, motor_id:int, torque:float) -> None:
        ...

    def set_sleep_duration(self, sleep_duration:float) -> None:
        ...

    def add_motor_to_zero(self, motor_id:int) -> None:
        ...

    def get_latest_feedback(self) -> dict[int, PyRobstrideMotorFeedback]:
        ...

    def toggle_pause(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def __repr__(self) -> str:
        ...

    def set_params(self, motor_id:int, params:PyRobstrideMotorControlParams) -> None:
        ...


