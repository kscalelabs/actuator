"""Defines the top-level API for the actuator package."""

from .bindings import (
    PyRobstrideActuator as RobstrideActuator,
    PyRobstrideActuatorConfig as RobstrideActuatorConfig,
    get_version,
)

__version__ = get_version()
 