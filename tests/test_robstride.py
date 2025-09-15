"""Simple tests for the Robstride actuators, without needing to be connected to a real actuator."""

from actuator import RobstrideActuator, RobstrideActuatorConfig


def test_robstride() -> None:
    supervisor = RobstrideActuator(
        ports=["stub"],
        py_actuators_config=[(1, RobstrideActuatorConfig(1))],
    )

    for _ in range(3):
        state = supervisor.get_actuators_state([1])
        assert isinstance(state, list)  # State is empty for now.
