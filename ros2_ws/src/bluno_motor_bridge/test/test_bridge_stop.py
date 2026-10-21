"""Check cancellation of firmware-local steps through zero ROS commands."""

from unittest.mock import Mock

from bluno_motor_bridge.bridge_node import BlunoMotorBridge
from std_msgs.msg import Int16MultiArray


def test_zero_command_interrupts_active_step():
    """Send an immediate serial STOP instead of awaiting the step watchdog."""
    bridge = Mock()
    bridge._step_active = True
    bridge._serial = Mock()
    bridge._limit = 255
    bridge._motor_sign = [1, 1]
    bridge._active_axes = [True, True]
    BlunoMotorBridge._on_command(bridge, Int16MultiArray(data=[0, 0]))
    bridge._serial.write.assert_called_once_with(b'STOP\n')
    bridge._serial.flush.assert_called_once()
    assert bridge._step_active is False
    assert bridge._commands == (0, 0)


def test_invalid_command_does_not_interrupt_step():
    """Reject malformed ROS commands without changing step state."""
    bridge = Mock()
    bridge._step_active = True
    BlunoMotorBridge._on_command(bridge, Int16MultiArray(data=[0]))
    bridge._serial.write.assert_not_called()
    assert bridge._step_active is True
