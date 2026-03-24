# Claude Session Notes

> **Purpose**: This file preserves cross-session context so Claude can pick up where we left off on any machine. Claude also stores persistent memory in its own memory directory.

---

## Session 1 — Desktop (2026-03-02)

### What Was Done
1. Read all project files — original code, new tests, water tests, notes, CLAUDE.md, competition manual PDF
2. Created `code_guide.md` — comprehensive function-by-function guide for all original code
3. Created `DepthSensor_NewTests/DepthSensor_DryTest_v2/DepthSensor_DryTest_v2.ino` — revamped dry test fixing 3 bugs in the old version
4. Created `DepthSensor_NewTests/PressureGraph/PressureGraph.ino` — updated mission station with 3-button control, manual mode toggle, data verification

### Key Findings
- WiFi teardown is an ESP32 ADC2 hardware limitation, not a MATE rule
- Water test code uses OLD ESP-NOW API signatures — won't compile on core 3.x
- MATE manual (pg. 22) requires data transmitted only after physical recovery

---

## Session 2 — Laptop/Lab (2026-03-02)

### What Was Done
1. Answered all 14 pending hardware questions (see Hardware Specs below)
2. Explained command codes (time=1.0/2.0/3.0), encoder concepts, WiFi-potentiometer cycle
3. Identified depth sensor noise as abnormal (±25cm spread, should be ±5cm) — see memory/debugging.md
4. Identified potentiometer disconnect failure mode (floating pin → descend) — already mitigated by manual mode toggle
5. Clarified L298N motor driver wiring and proper PWM usage (EN pin for speed, IN pins for direction)

### Hardware Answers Received
- ESP32 core v3.3.7, Arduino IDE v2.3.8, ESP32Encoder v0.12.0
- Depth sensor: DFROBOT GL-136 (4-20mA, 0-5m)
- Motor: gobuilda 5203 Yellow Jacket, lead screw, 11 rev full stroke
- Syringe: 40-320ml range (~280ml usable)
- Battery: MightyMax ML-5-12 (12V 5Ah AGM)
- Motor driver: L298N, GPIO 18→IN3, GPIO 19→IN4, ENB pin TBD
- Float: 27in x 8in cylinder, external weights for neutral buoyancy
- Pool: 1.5m freshwater, no water testing yet
- D23 wired on mission station

### Pending / Next Steps
- **Waiting on**: Mechanical team to wire L298N ENB pin for PWM speed control
- **Then**: Update DryTest_v2 motor functions: digitalWrite on IN3/IN4 for direction, ledcWrite on ENB for speed
- **Variable speed logic already written** — calculateApproachSpeed() and calculateCorrectionSpeed() just need rewiring
- **Depth sensor noise**: needs investigation after dry test; software filtering (median/moving average) recommended
- **Half-depth test version**: may be needed for 1.5m pool (current targets: 2.5m/0.4m)
- **Potentiometer issues**: deferred, hardware improvements planned

---

## Key Technical Reference

- **Float MAC**: F8:B3:B7:3E:FE:88 | **Station MAC**: 8C:4F:00:10:07:34
- **State machine**: IDLE → PRE_DIVE_TX → DESCENDING → HOLD_DEEP → ASCENDING → HOLD_SHALLOW → COMPLETE → RETURNING_HOME → TRANSMITTING
- **Commands**: time=1.0 (start dive), time=2.0 (manual motor), time=3.0 (transmit data)
- **Scoring**: 7 sequential packets at 5s intervals, ±33cm tolerance during 30s holds
- **Sensor offsets**: 25cm from bottom, 42.4cm from top of float
- **Encoder**: 2600 PPR half-quad, 11 revolutions full stroke = 28,600 pulses
- **Power limits**: 12 VDC max, 5 amps max, NiMH/AGM only, single fuse
- **L298N wiring**: GPIO 18→IN3 (direction A), GPIO 19→IN4 (direction B), ENB→TBD (PWM speed)
