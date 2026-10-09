"""Unit tests for the ROS-independent serial protocol codec."""

from bluno_motor_bridge.protocol import (
    adc_to_current_a,
    CURRENT_UNAVAILABLE_MA,
    encode_command,
    encode_step_axis,
    encode_stop,
    encode_zero_encoders,
    parse_state,
    ProtocolError,
)
import pytest


def test_adc_to_current_for_acs712_30a() -> None:
    """One ADC count is about 74 mA with nominal 5 V and 66 mV/A."""
    assert adc_to_current_a(517, 517.0, 5.0, 0.066) == 0.0
    assert adc_to_current_a(518, 517.0, 5.0, 0.066) == pytest.approx(
        0.07405, rel=1e-3
    )
    assert adc_to_current_a(516, 517.0, 5.0, 0.066, -1) == pytest.approx(
        0.07405, rel=1e-3
    )
    assert adc_to_current_a(
        518, 517.0, 5.0, 0.066, deadband_adc_counts=1.0
    ) == 0.0
    assert adc_to_current_a(
        519, 517.0, 5.0, 0.066, deadband_adc_counts=1.0
    ) == pytest.approx(0.1481, rel=1e-3)


def test_encode_command() -> None:
    assert encode_command(152, (80, -40)) == b'CMD,152,80,-40\n'
    assert encode_stop() == b'STOP\n'
    assert encode_step_axis(7, 1, -5, 120) == b'STEP_AXIS,7,1,-5,120\n'
    assert encode_zero_encoders() == b'ZERO_ENC\n'


@pytest.mark.parametrize(
    'sequence,axis,counts,pwm',
    [(-1, 0, 1, 1), (0, 2, 1, 1), (0, 0, 0, 1),
     (0, 0, 51, 1), (0, 0, 1, 0), (0, 0, 1, 256)],
)
def test_encode_step_axis_rejects_invalid_values(
    sequence, axis, counts, pwm
) -> None:
    with pytest.raises(ProtocolError):
        encode_step_axis(sequence, axis, counts, pwm)


@pytest.mark.parametrize(
    'sequence,commands',
    [(-1, (0, 0)), (0, (256, 0)), (0, (0, -256)), (0, (0,)), (0, (1.2, 0))],
)
def test_encode_command_rejects_invalid_values(sequence, commands) -> None:
    with pytest.raises(ProtocolError):
        encode_command(sequence, commands)


def test_parse_state() -> None:
    packet = parse_state(
        b'STATE,152,38124930,12584,-3402,80,-40,522,507,410,-120,0\r\n'
    )
    assert packet.sequence == 152
    assert packet.mcu_time_us == 38124930
    assert packet.encoder_counts == (12584, -3402)
    assert packet.applied_commands == (80, -40)
    assert packet.raw_adc == (522, 507)
    assert packet.current_ma == (410, -120)
    assert packet.flags == 0


def test_current_unavailable_sentinel_is_valid_telemetry() -> None:
    packet = parse_state(
        'STATE,1,2,0,0,512,0,-2147483648,-2147483648,24'
    )
    assert packet.current_ma == (
        CURRENT_UNAVAILABLE_MA,
        CURRENT_UNAVAILABLE_MA,
    )
    assert packet.applied_commands is None


@pytest.mark.parametrize(
    'line',
    [
        '',
        'DEBUG,hello',
        'STATE,1,2,3',
        'STATE,1,2,3,4,1024,0,0,0,0',
        'STATE,1,2,3,4,0,-1,0,0,0',
        'STATE,a,2,3,4,0,0,0,0,0',
    ],
)
def test_parse_state_rejects_malformed_packets(line) -> None:
    with pytest.raises(ProtocolError):
        parse_state(line)
