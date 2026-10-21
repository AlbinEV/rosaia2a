# OPEN/CLOSE handoff — 20 October 2026

## Update after author reported tendon tension

Later bounded exploration produced two actual contraction/release cycles on
axis 0, observed on camera ARM 2. Close positions were q=19 and q=15,
with inward tip changes about 8.5 and 9.1 mm. Return tip differences were
about 2.9 and 3.8 mm. A local pilot candidate is OPEN=0, CLOSE=16 with
four-count tolerance, contraction PWM 150, release PWM 110, five-count
firmware steps. This is a single-axis result, not a verified dual-arm demo.
Axis 1 still has no confirmed visible contraction or camera association.
See `data/control_tests/open_close_partial_20261020.json` and the two
workflow reports referenced there. These later results supersede the
earlier statement below that no CLOSE candidate exists at all.

## Physical reference

The author explicitly confirmed the current physical pose as HOME immediately
before the open/close search. Encoders were zeroed successfully at that pose.
Do not reuse earlier neural calibrations against this new encoder zero.

## Confirmed mapping and unresolved item

- Physical axis 0 / shield M2 moves camera ARM 2. Confirmed by the author
  after the timed PWM 150 test, with visible ARM 2 movement.
- Axis 1 / shield M1 is expected to correspond to camera ARM 1, but its
  visible motion has NOT been confirmed. Do not mark the full mapping verified.
- Existing software associates axis 0 with arm_1. This is incorrect for the
  observed setup. Before enabling autonomous operation, update dashboard,
  recorder, velocity-reference routing, learning metadata and UI telemetry
  together; do not change firmware encoder signs just to rename camera arms.
- Previous small-excursion neural models used the old association and zero.
  They are not valid calibrations for OPEN/CLOSE in this configuration.

## Latest checks and stopping state

- Axis 0, timed PWM 150: encoder reached approximately 67 counts. The
  travel guard interrupted the command, with overshoot after STOP. Returned
  with bounded five-count requests at PWM 110, ending at raw encoder 3.
- Axis 1, timed PWM 200, requested duration 0.08 seconds: encoder reached
  approximately -72. Travel guard interrupted; visible ARM 1 motion was not
  established. Returned with bounded five-count requests at PWM 120, ending
  at raw encoder -4.
- Both outputs were stopped after each check. No verified CLOSE pose exists.
- The web interface has one disabled OPEN/CLOSE control and an active STOP.
  Do not silently enable guessed poses.

## Reproducible evidence

- `data/sessions/open_close_20261020_200718`: axis 0 timed test.
- `data/sessions/open_close_20261020_200741`: axis 0 return.
- `data/sessions/open_close_20261020_200906`: axis 1 timed test.
- `data/sessions/open_close_20261020_200932`: axis 1 return.
- Matching workflow directories contain result JSON and recorder logs.

The earlier 30-count attempt was rejected by the bridge's 10-count step
limit: absence of motion there is NOT evidence of motor stall. Future
diagnostics must distinguish rejected commands from accepted PWM telemetry.
Timed pulses showed substantial encoder overshoot; avoid repeating them at
these settings for a small-excursion demo. Prefer firmware-bounded requests
and visual confirmation before selecting OPEN/CLOSE values.
