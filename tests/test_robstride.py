"""Simple tests for the Robstride actuators, without needing to be connected to a real actuator."""

from actuator import RobstrideActuator, RobstrideActuatorConfig, StubTransportWrapper


def test_robstride() -> None:
    # Create transport object instead of using port string
    transport = StubTransportWrapper()
    data = [0x7f, 0xfe, 0x80, 0x73, 0x7f, 0xff, 0x01, 0x18]
    transport.send(1, data)
    recv_id, recv_state = transport.recv()
    assert recv_id == 1
    assert recv_state == data

    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(1, RobstrideActuatorConfig(1))],
    )

    for _ in range(3):
        state = supervisor.get_actuators_state([1])
        assert isinstance(state, list)  # State is empty for now.
