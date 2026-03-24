# MATE Float 2026 - DepthSensor_NewTests

## Project Overview
Vertical profiling float for MATE ROV competition. Two ESP32-WROOM-32 boards communicate via ESP-NOW:
- **Float** (`DepthSensor_DryTest/DepthSensor_DryTest.ino`): Controls buoyancy engine (syringe + motor), reads depth sensor, runs dive state machine
- **Mission Station** (`PressureGraph/PressureGraph.ino`): Operator controls — buttons + potentiometer, receives/displays data

## Hardware
- Float: ESP32 with depth sensor (pin 25), motor H-bridge (pins 18/19), encoder (pins 17/4), LEDs (pins 2/15)
- Mission Station: ESP32 with 3 buttons (D21=START, D22=TRANSMIT, D23=MANUAL toggle), potentiometer (D4)
- Buoyancy engine: syringe driven by motor, 11 full revolutions from empty to full (2600 pulses/rev, 28600 pulses total)
- Sensor offsets: sensor-to-bottom 25cm, sensor-to-top 42.4cm, float height 67.4cm

## Changes Made This Session

### Safety: Hard encoder limits (DepthSensor_DryTest.ino)
- `moveFloatDown()` and `moveFloatUp()` check encoder position before applying PWM
- ENCODER_MIN_COUNT = -28600 (11 revolutions, max extension/descent)
- ENCODER_MAX_COUNT = 0 (home position, max retraction/ascent)
- Protects ALL code paths: dive logic, hold corrections, manual control
- Removed `buoyEnc.clearCount()` from state transitions — only cleared in `setup()` so absolute position is always tracked
- Removed per-state encoder checks (now handled globally in motor functions)

### Manual control mode (PressureGraph.ino)
- Added BUTTON_MANUAL on GPIO 23 — toggles manual mode on/off
- Potentiometer only sends motor commands when manual mode is explicitly enabled
- No motor commands sent on boot (removed `initialLoop` auto-send)
- Disabling manual mode immediately sends stop command
- Dive start blocked while manual mode is active (prints warning)

### Auto-return to home (DepthSensor_DryTest.ino)
- Added STATE_RETURNING_HOME to ProfileState enum
- When transmit command (3.0) is received, motor drives back to encoder position 0 before transmitting data
- Uses proportional speed (full speed far away, slower near home, stops within ±50 pulse deadband)

### Dry test simulation fixes (DepthSensor_DryTest.ino)
- Simulation timer now uses `profileStartTime` (set on START command), not boot time
- Simulation only produces non-zero depth during active dive states
- Fixed simulation to use SENSOR target values (sensorTargetDeep=2.25m, sensorTargetShallow=0.824m) instead of raw physical depths (2.5m, 0.4m) — the state machine compares against sensor-offset ranges

## What Still Needs Testing
- Full dry test dive cycle: descend → hold deep 30s → ascend → hold shallow 30s → complete (×2 profiles)
- Return-to-home after dive completes (press D22 transmit)
- Data transmission to mission station after return-to-home
- Verify encoder limits are hit and respected during dry test (motor should stop, not stall)
- The dry test simulation is time-based and independent of motor position — the motor will run even though the simulated depth changes on its own timeline. This is expected for testing the state machine logic.

## Key Architecture Notes
- Float tears down WiFi during dive (interference with depth sensor). It's unreachable until dive completes and transmit is requested.
- ESP-NOW commands: time=1.0 (start dive), time=2.0 (potentiometer motor control), time=3.0 (transmit data)
- Sensor depth targets are offset from physical depths: sensorTargetDeep = TARGET_DEEP - SENSOR_TO_BOTTOM_OFFSET, sensorTargetShallow = TARGET_SHALLOW + SENSOR_TO_TOP_OFFSET
