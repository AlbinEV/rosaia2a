"""Pure-Python codec for the line-oriented Bluno serial protocol."""

from dataclasses import asdict, dataclass


AXIS_COUNT = 2
MIN_FIRMWARE_COMMAND = -255
MAX_FIRMWARE_COMMAND = 255
CURRENT_UNAVAILABLE_MA = -(2**31)


def adc_to_current_a(
    raw_adc: int,
    zero_adc: float,
    adc_reference_v: float,
    sensitivity_v_per_a: float,
    polarity: int = 1,
    deadband_adc_counts: float = 0.0,
) -> float:
    """Convert one ACS712 ADC sample to signed current in ampere."""
    if not 0 <= raw_adc <= 1023:
        raise ValueError('raw ADC must be in [0, 1023]')
    if not 0.0 <= zero_adc <= 1023.0:
        raise ValueError('zero ADC must be in [0, 1023]')
    if adc_reference_v <= 0.0 or sensitivity_v_per_a <= 0.0:
        raise ValueError('ADC reference and sensitivity must be positive')
    if polarity not in (-1, 1):
        raise ValueError('current polarity must be -1 or 1')
    if deadband_adc_counts < 0.0:
        raise ValueError('ADC deadband must be non-negative')
    if abs(raw_adc - zero_adc) <= deadband_adc_counts:
        return 0.0
    volts_per_count = adc_reference_v / 1023.0
    return (
        polarity
        * (raw_adc - zero_adc)
        * volts_per_count
        / sensitivity_v_per_a
    )


class ProtocolError(ValueError):
    """Raised when a serial packet violates the protocol contract."""


@dataclass(frozen=True)
class StatePacket:
    """Decoded telemetry produced by the Bluno firmware."""

    sequence: int
    mcu_time_us: int
    encoder_counts: tuple[int, int]
    applied_commands: tuple[int, int] | None
    raw_adc: tuple[int, int]
    current_ma: tuple[int, int]
    flags: int

    def as_dict(self) -> dict:
        """Return a JSON-serializable representation."""
        return asdict(self)


def encode_command(sequence: int, commands: tuple[int, int]) -> bytes:
    """Encode one command after strict type and range validation."""
    if len(commands) != AXIS_COUNT:
        raise ProtocolError(f'expected {AXIS_COUNT} commands, got {len(commands)}')
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ProtocolError('sequence must be a non-negative integer')

    checked = []
    for command in commands:
        if isinstance(command, bool) or not isinstance(command, int):
            raise ProtocolError('commands must be integers')
        if not MIN_FIRMWARE_COMMAND <= command <= MAX_FIRMWARE_COMMAND:
            raise ProtocolError(f'command {command} is outside [-255, 255]')
        checked.append(command)

    return f'CMD,{sequence},{checked[0]},{checked[1]}\n'.encode('ascii')


def encode_stop() -> bytes:
    """Encode the unconditional stop command."""
    return b'STOP\n'


def encode_zero_encoders() -> bytes:
    """Encode a mechanical-home encoder reset."""
    return b'ZERO_ENC\n'


def encode_step_axis(
    sequence: int, axis: int, signed_count_limit: int, pwm: int
) -> bytes:
    """Encode one locally bounded calibration move."""
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ProtocolError('sequence must be a non-negative integer')
    if isinstance(axis, bool) or not isinstance(axis, int) or axis not in (0, 1):
        raise ProtocolError('axis must be 0 or 1')
    if (
        isinstance(signed_count_limit, bool)
        or not isinstance(signed_count_limit, int)
        or signed_count_limit == 0
        or abs(signed_count_limit) > 50
    ):
        raise ProtocolError('signed_count_limit must be in [-50, 50] excluding 0')
    if (
        isinstance(pwm, bool)
        or not isinstance(pwm, int)
        or not 1 <= pwm <= MAX_FIRMWARE_COMMAND
    ):
        raise ProtocolError('pwm must be in [1, 255]')
    return (
        f'STEP_AXIS,{sequence},{axis},{signed_count_limit},{pwm}\n'
    ).encode('ascii')


def parse_state(line: str | bytes) -> StatePacket:
    """Decode a STATE packet and reject malformed or out-of-range fields."""
    if isinstance(line, bytes):
        try:
            line = line.decode('ascii')
        except UnicodeDecodeError as exc:
            raise ProtocolError('packet is not ASCII') from exc

    fields = line.strip().split(',')
    if len(fields) not in (10, 12) or fields[0] != 'STATE':
        raise ProtocolError('expected version-2 or version-3 STATE packet')

    try:
        values = tuple(int(value, 10) for value in fields[1:])
    except ValueError as exc:
        raise ProtocolError('STATE contains a non-integer field') from exc

    if len(values) == 9:
        sequence, mcu_time_us, enc0, enc1, adc0, adc1, ma0, ma1, flags = values
        applied_commands = None
    else:
        (
            sequence, mcu_time_us, enc0, enc1, command0, command1,
            adc0, adc1, ma0, ma1, flags,
        ) = values
        applied_commands = (command0, command1)
    if sequence < 0 or mcu_time_us < 0:
        raise ProtocolError('sequence and MCU time must be non-negative')
    if not (0 <= adc0 <= 1023 and 0 <= adc1 <= 1023):
        raise ProtocolError('ADC value is outside the ATmega328 10-bit range')
    if flags < 0:
        raise ProtocolError('flags must be non-negative')
    if applied_commands is not None and any(
        not MIN_FIRMWARE_COMMAND <= value <= MAX_FIRMWARE_COMMAND
        for value in applied_commands
    ):
        raise ProtocolError('applied command is outside [-255, 255]')

    return StatePacket(
        sequence=sequence,
        mcu_time_us=mcu_time_us,
        encoder_counts=(enc0, enc1),
        applied_commands=applied_commands,
        raw_adc=(adc0, adc1),
        current_ma=(ma0, ma1),
        flags=flags,
    )
