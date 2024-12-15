"""Defines the top-level API for the actuator package."""

from .bindings import (
    PyRobstrideActuator as RobstrideActuator,
    PyRobstrideActuatorCommand as RobstrideActuatorCommand,
    PyRobstrideActuatorState as RobstrideActuatorState,
    PyRobstrideConfigureRequest as RobstrideConfigureRequest,
    get_version,
)

__version__ = get_version()
