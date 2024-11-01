"""Defines the top-level API for the actuator package."""

from .bindings import (
    PyRobstrideMotorConfig as RobstrideMotorConfig,
    PyRobstrideMotorControlParams as RobstrideMotorControlParams,
    PyRobstrideMotorFeedback as RobstrideMotorFeedback,
    PyRobstrideMotors as RobstrideMotors,
    PyRobstrideMotorsSupervisor as RobstrideMotorsSupervisor,
    PyRobstrideMotorType as RobstrideMotorType,
    get_version,
)

__version__ = get_version()
