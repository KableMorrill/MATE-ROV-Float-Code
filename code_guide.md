# MATE Float Code Guide

This document explains every function in the **original** DepthSensor (float) and PressureGraph
(mission station) code from last year, what it does, why it exists, and how it relates to the
ESP-32 platform. It also explains the key changes made in the newer test versions.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [ESP-NOW Primer](#esp-now-primer)
3. [Original Float Code (DepthSensor/DepthSensor.ino)](#original-float-code)
4. [Original Mission Station Code (PressureGraph/PressureGraph.ino)](#original-mission-station-code)
5. [New Test Versions — What Changed](#new-test-versions)
6. [ESP32 Arduino Core 3.x API Changes](#esp32-core-3x-changes)
7. [Key Concepts to Understand](#key-concepts)

---

## System Overview

Two ESP32-WROOM-32 boards communicate wirelessly using **ESP-NOW** (a low-latency,
peer-to-peer protocol built into the ESP32). No WiFi router is needed.

```
+-------------------+     ESP-NOW (wireless)     +-------------------+
|   MISSION STATION |  <---------------------->  |      FLOAT        |
|   (PressureGraph) |                            |   (DepthSensor)   |
|                   |                            |                   |
|  - Start button   |  -- "start" command -->    |  - Depth sensor   |
|  - Potentiometer  |  -- "motor" commands -->   |  - Motor + encoder|
|  - Serial display |  <-- depth data --------   |  - LEDs           |
+-------------------+                            +-------------------+
```

**Communication flow:**
1. Station sends a START command (time=1.0) to the float
2. Float tears down WiFi (to avoid interference with the analog depth sensor)
3. Float performs its dive profile autonomously, recording data
4. Float is recovered to the surface
5. Station sends a TRANSMIT command (time=3.0)
6. Float re-enables WiFi and sends all recorded data back

**Why tear down WiFi during diving?**
The ESP32's WiFi radio creates electromagnetic interference that degrades analog sensor
readings. The ADC (Analog-to-Digital Converter) on the ESP32 is notoriously noisy, and
WiFi makes it worse. By disabling WiFi during the dive, the depth sensor readings are
more accurate.

---

## ESP-NOW Primer

ESP-NOW is a protocol developed by Espressif (the ESP32 manufacturer). Key facts:

- **No router needed** — devices communicate directly with each other
- **Uses MAC addresses** — each ESP32 has a unique MAC address burned into it
- **250 bytes max per packet** — your data structures must fit in this
- **Low latency** — messages arrive in milliseconds
- **Requires WiFi radio** — ESP-NOW runs on top of the WiFi hardware, so WiFi mode
  must be enabled (WIFI_STA), but you don't need to connect to a network

### How ESP-NOW works in this project:

1. **Setup**: Call `WiFi.mode(WIFI_STA)` then `esp_now_init()` to start ESP-NOW
2. **Register peer**: Tell ESP-NOW the MAC address of the other board
3. **Send**: Call `esp_now_send()` with the peer's MAC and your data as a byte array
4. **Receive**: Register a callback function that ESP-NOW calls whenever data arrives
5. **Teardown**: Call `esp_now_deinit()`, `WiFi.disconnect()`, `WiFi.mode(WIFI_OFF)`

### MAC Addresses in this project:
- **Mission Station**: `8C:4F:00:10:07:34`
- **Float**: `F8:B3:B7:3E:FE:88`

Each board stores the OTHER board's MAC address as `broadcastAddress[]`. The float
sends TO the station's MAC; the station sends TO the float's MAC.

---

## Original Float Code

**File:** `DepthSensor/DepthSensor.ino`

### Constants and Pin Definitions (lines 1-19)

```cpp
#define DEPTH_PIN 25        // Analog input from depth sensor
#define IN1 18              // Motor H-bridge input 1
#define IN2 19              // Motor H-bridge input 2
#define ENC1 17             // Encoder channel A
#define ENC2 4              // Encoder channel B
#define PULSES_PER_REV 2600 // Encoder resolution
#define TARGET_DIVE_PULSES PULSES_PER_REV * -8  // = -20800 (8 revolutions down)
#define TARGET_SURFACE_PULSES 0                  // Home position
#define DATA_PER_PACKET 30  // Data points per ESP-NOW transmission
#define SECONDS_UNDERWATER 120  // Total recording time
#define NUM_DIVES 2
#define DIVE_FLOOR .5       // Depth (meters) that triggers ascent
#define DIVE_CEILING 0      // Depth (meters) that triggers next descent
#define B_LED 2             // Onboard LED (GPIO 2 on most ESP32 boards)
```

**What to understand:**
- `IN1`/`IN2` control motor direction via an H-bridge driver. Setting one HIGH and the
  other LOW spins the motor one way; reversing them spins it the other way. Both LOW = stop.
- The encoder tracks motor position. Negative counts = motor has turned in the "descend"
  direction. The code uses this to limit how far the motor can drive the syringe.
- `DIVE_FLOOR` and `DIVE_CEILING` were originally 3.0m and 0.666m but were changed to
  0.5m and 0m for shallow pool testing. These are the depth thresholds that trigger state
  changes in the original logic.

### Data Structure (lines 29-32)

```cpp
typedef struct struct_message {
    float time;
    float depth;
} struct_message;
```

This is the packet format for ESP-NOW communication. Both boards must define the SAME
structure so `memcpy()` can correctly interpret the bytes. In the original code, it only
carries time and depth. The new versions add `companyNum[8]` and `pressure_kpa`.

### Global Variables (lines 35-44)

- `myData[SECONDS_UNDERWATER]` — Array storing one reading per second (120 total)
- `dataPacket[DATA_PER_PACKET]` — Buffer for sending 30 readings at a time
- `seconds` — Counter for how many readings have been taken (starts at -1, incremented
  before each reading, so first reading is index 0)
- `currentArray` — Tracks which chunk of 30 readings to send next
- `started` — Flag set to true when the dive command is received
- `divesRemaining` — Counts down from NUM_DIVES
- `descending` — True when syringe is pushing water out (float sinks)
- `motorsMoving` — True when motor is actively running
- `buoyEnc` — ESP32Encoder object that tracks motor position via hardware interrupts
- `wifiOn` / `takeDownWifi` — Flags for WiFi state management

### `setup()` (lines 47-65)

Standard Arduino setup:
1. `Serial.begin(9600)` — Initializes serial monitor for debugging at 9600 baud
2. `pinMode(IN1/IN2, OUTPUT)` — Configures motor control pins
3. `pinMode(B_LED, OUTPUT)` — Configures debug LED
4. `buoyEnc.attachHalfQuad(ENC1, ENC2)` — Tells the encoder library which pins to
   monitor. "Half quad" means it counts on one edge of one channel (vs full quadrature
   which counts both edges of both channels = 4x resolution). With 2600 PPR in half-quad,
   you get 2600 counts per revolution.
5. `buoyEnc.clearCount()` — Sets encoder position to 0 (this is "home")
6. `setupPeer()` — Initializes ESP-NOW and registers the mission station as a peer

### `setupPeer()` (lines 67-103)

Sets up ESP-NOW communication:

1. `WiFi.mode(WIFI_STA)` — Puts WiFi into Station mode (required for ESP-NOW)
2. `esp_now_init()` — Initializes the ESP-NOW protocol
3. `esp_now_register_send_cb(OnDataSent)` — Registers a callback function that gets
   called AFTER every send, telling you if it succeeded or failed
4. `esp_now_register_recv_cb(OnDataRecv)` — Registers a callback function that gets
   called whenever data is RECEIVED from another ESP-NOW device
5. Sets up `peerInfo` with the station's MAC address, channel 0, no encryption
6. `esp_now_add_peer()` — Registers the station as a known peer

**Returns:** `true` if any step failed, `false` on success. This inverted convention is
used throughout the code — check the return value and bail out if true.

### `DEFCON1()` (lines 105-137)

Safely tears down all ESP-NOW and WiFi:

1. Checks if peers exist and removes them (`esp_now_del_peer`)
2. Deinitializes ESP-NOW (`esp_now_deinit`)
3. Disconnects WiFi and turns off the radio (`WiFi.mode(WIFI_OFF)`)
4. Sets `wifiOn = false`

**Why "DEFCON1"?** Just a dramatic name for "shut everything down." This exists because
ESP-NOW can't be re-initialized cleanly without fully tearing it down first. If you try
to call `esp_now_init()` while it's already initialized, or add a peer that already
exists, you get errors.

**Why do this at all?** Two reasons:
1. WiFi interference with the ADC (depth sensor analog readings)
2. The ESP32 can't do `analogRead()` on certain pins while WiFi is active (this is a
   known ESP32 limitation — WiFi uses ADC2, and ADC2 channels conflict with WiFi)

**Important:** Pin 25 (DEPTH_PIN) is on ADC2. This means `analogRead(25)` will return
garbage while WiFi is on. This is why the float tears down WiFi before diving.

### `sendData()` (lines 140-170)

Sends recorded depth data back to the station in chunks of 30:

1. Calls `setupPeer()` to re-enable WiFi and ESP-NOW
2. Clears the `dataPacket` buffer
3. Copies the next 30 data points from `myData` into `dataPacket`
4. Sends via `esp_now_send()`
5. Increments `currentArray` on success

**Note:** This only sends ONE packet per call. In the original code, it's called once
when the dive completes, so only the first 30 data points are sent. This is a known
limitation of the original code.

### `OnDataSent()` (line 173-175)

Callback that fires after each ESP-NOW send. Just prints success/failure.

### `OnDataRecv()` (lines 178-205)

Callback that fires when the float receives data from the station. This is the
**command handler**:

- `time == 1.0` → **START command**: Resets all state, clears data arrays, sets
  `takeDownWifi = true` so the main loop will tear down WiFi and begin diving
- `time == 2.0` → **MANUAL MOTOR command**: The `depth` field encodes direction:
  - `-1.0` = move up (expel water, ascend)
  - `0.0` = stop
  - `1.0` = move down (take on water, descend)

**Why use `time` as a command code?** The original code repurposes the data structure
fields as a simple command protocol. `time=1.0` means "start", `time=2.0` means
"motor control." The new code adds `time=3.0` for "transmit data."

### `moveFloatDown()` (lines 208-213)

Sets motor to descend (take on water):
- IN1=LOW, IN2=HIGH → motor spins in one direction
- Sets `motorsMoving=true`, `descending=true`

### `moveFloatUp()` (lines 216-223)

Sets motor to ascend (expel water):
- IN1=HIGH, IN2=LOW → motor spins in other direction
- Sets `motorsMoving=true`, `descending=false`
- **Has a 1-second delay** — this was likely added to give the motor time to start
  before continuing. This is problematic because `delay()` blocks the entire CPU.

### `stopFloat()` (lines 226-230)

Stops the motor:
- IN1=LOW, IN2=LOW → motor brakes (or coasts, depending on H-bridge)
- Sets `motorsMoving=false`

### `readDepthSensor()` (lines 233-240)

Reads the analog depth sensor and converts to meters:

```cpp
float analogReading = analogRead(DEPTH_PIN);    // 0-4095 (12-bit ADC)
float voltage = (analogReading / 4095.0) * 3.3; // Convert to voltage (0-3.3V)
float mAmps = (voltage / 150.0) * 1000;         // Convert to milliamps (Ohm's law: I=V/R)
float depth = (312.5 * (mAmps - 4)) / 1000;     // Convert mA to depth
```

**How this works:**
1. The depth sensor is a **4-20mA pressure transducer**
2. It outputs 4mA at 0 meters and 20mA at 5 meters (typical range)
3. A 150-ohm resistor converts the current to a voltage that the ESP32 can read
4. At 4mA: voltage = 4mA * 150Ω = 0.6V → depth = 0m
5. At 20mA: voltage = 20mA * 150Ω = 3.0V → depth = 5m
6. The formula `(312.5 * (mAmps - 4)) / 1000` linearly maps 4-20mA to 0-5m

**Why does it read negative on the surface?** The ADC has some offset error, and the
sensor may not output exactly 4.00mA at atmospheric pressure. A reading of -0.12m means
the sensor is outputting slightly less than 4mA.

### `loop()` (lines 242-297)

The main control loop, runs continuously:

**Every second** (lines 256-268):
- If the dive has started and WiFi is off, read the depth sensor
- Store time and depth in `myData[seconds]`

**Depth-based state transitions** (lines 273-288):
- If descending and depth >= DIVE_FLOOR (0.5m): turn on LED, start ascending
- If ascending and depth <= DIVE_CEILING (0m): start next dive or finish

**Encoder-based motor limits** (lines 290-296):
- If descending and encoder <= TARGET_DIVE_PULSES (-20800): stop motor
- If ascending and encoder >= TARGET_SURFACE_PULSES (0): stop motor

**Critical limitation of the original loop:** The depth checks and encoder checks run
every single loop iteration (thousands of times per second), but the depth reading only
updates once per second. This means the depth-based transitions use the most recent
1-second reading, which can be stale.

---

## Original Mission Station Code

**File:** `PressureGraph/PressureGraph.ino`

### Constants and Pins

```cpp
#define BUTTON_START 21   // Button to start dive sequence
#define PEN_IN 4          // Potentiometer analog input
```

**Note:** The original station only has ONE button and ONE potentiometer. The new version
adds BUTTON_TRANSMIT (D22) and BUTTON_MANUAL (D23).

### `setup()` (lines 30-40)

Simple: init serial, configure button pin with internal pull-up resistor.

`INPUT_PULLUP` means the pin reads HIGH when the button is NOT pressed, and LOW when
pressed (button connects pin to ground). This is why button logic checks for LOW = pressed.

### `setupPeer()` (lines 42-78)

Identical to the float's version, except `broadcastAddress` is the FLOAT's MAC address
(the station talks TO the float).

### `DEFCON1()` (lines 80-112)

Identical to the float's version — full WiFi/ESP-NOW teardown.

### `sendStartSignal()` (lines 115-130)

Sends a packet with `time=1.0` to tell the float to start diving.

### `sendPotentiometerReading()` (lines 133-147)

Sends a packet with `time=2.0` and the potentiometer value (-1, 0, or 1) in the `depth`
field to control the motor manually.

### `OnDataSent()` (lines 150-159)

After sending, decides whether to tear down WiFi. The `connTakeDownOverride` flag
prevents teardown when we're about to send another command immediately (like sending
a stop command followed by a start command).

### `OnDataRecv()` (lines 162-171)

Receives depth data from the float and prints it as CSV (`time,depth`). In the original
code, this just dumps data to the serial monitor.

### Debounce Functions (lines 174-193)

`debounceBool()` and `debounceInt()` prevent noisy button/potentiometer readings from
causing false triggers. They work by requiring the reading to be stable for `debounceTime`
milliseconds before accepting it as valid.

### `readPotentiometer()` (lines 196-209)

Reads the potentiometer voltage and maps it to three zones:
- Below 1.1V → `-1` (move up)
- 1.1V to 2.2V → `0` (stop)
- Above 2.2V → `1` (move down)

This is a simple three-position control: left = up, center = stop, right = down.

### `loop()` (lines 212-281)

Main loop handles:

1. **WiFi teardown** if flagged
2. **Potentiometer** reading (only when WiFi is off — remember ADC2 conflict)
3. **Debouncing** of potentiometer value
4. **Sending potentiometer** commands when value changes
5. **Start button** logic: on press, send a stop command first, wait for it to complete,
   then send the start command

**The `while(!takeDownWifi)` busy-wait (line 269):** This blocks the entire CPU until the
OnDataSent callback sets `takeDownWifi = true`. This is a crude way to ensure the stop
command is fully sent before sending the start command. It works but is fragile.

---

## New Test Versions

### Key Changes in DepthSensor_DryTest

1. **State machine** replaces simple if/else depth checks:
   ```
   IDLE → PRE_DIVE_TX → DESCENDING → HOLD_DEEP → ASCENDING →
   HOLD_SHALLOW → (repeat for profile 2) → COMPLETE → RETURNING_HOME → TRANSMITTING
   ```

2. **PWM motor speed control** using `ledcAttach()` instead of `digitalWrite()`:
   - Allows variable speed (0-255) instead of just on/off
   - `calculateApproachSpeed()` ramps speed down as float nears target depth
   - `calculateCorrectionSpeed()` uses gentler speeds during hold corrections

3. **Hard encoder safety limits** in motor functions:
   - `ENCODER_MIN_COUNT = -28600` (11 revolutions, max syringe extension)
   - `ENCODER_MAX_COUNT = 0` (home position)
   - Motor functions check these BEFORE applying power

4. **Sensor offset calculations** for competition scoring:
   - Deep hold: judges measure BOTTOM of float (sensor + 25cm)
   - Shallow hold: judges measure TOP of float (sensor - 42.4cm)

5. **Time-based depth simulation** for dry testing (the part that's broken)

### Key Changes in PressureGraph (New)

1. **Three buttons**: START (D21), TRANSMIT (D22), MANUAL toggle (D23)
2. **Updated data structure** with `companyNum`, `pressure_kpa`
3. **Data verification**: counts sequential in-range packets for judge review
4. **Manual mode safety**: potentiometer only sends when explicitly enabled

---

## ESP32 Core 3.x Changes

The ESP32 Arduino core version 3.x (based on IDF v5.x) made breaking API changes:

### ESP-NOW Callbacks — OLD (core 2.x):
```cpp
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
```

### ESP-NOW Callbacks — NEW (core 3.x):
```cpp
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
```

### LEDC (PWM) — OLD (core 2.x):
```cpp
ledcSetup(channel, freq, resolution);
ledcAttachPin(pin, channel);
ledcWrite(channel, duty);
```

### LEDC (PWM) — NEW (core 3.x):
```cpp
ledcAttach(pin, freq, resolution);  // combines setup + attach
ledcWrite(pin, duty);               // uses pin directly, not channel
```

**This is why your code didn't compile.** The DryTest file was already updated to use
the new API, but the WaterTest files still use the old API.

---

## Key Concepts

### How the Buoyancy Engine Works

```
Motor turns → Lead screw moves → Syringe plunger moves → Water enters/exits float

Motor CW:  Plunger pulls in  → Water enters  → Float gets heavier → DESCENDS
Motor CCW: Plunger pushes out → Water exits   → Float gets lighter → ASCENDS
```

The encoder tracks how far the motor has turned, which tells you how much water is in
the syringe. Encoder position = syringe position = buoyancy state.

### Why WiFi and analogRead() Conflict

The ESP32 has two ADCs:
- **ADC1** (GPIOs 32-39): Works anytime
- **ADC2** (GPIOs 0, 2, 4, 12-15, 25-27): **Cannot be used while WiFi is active**

Your depth sensor is on **GPIO 25 (ADC2)**. This means:
- WiFi ON → `analogRead(25)` returns garbage
- WiFi OFF → `analogRead(25)` works correctly

This is why the code tears down WiFi before diving and re-enables it for data transmission.

**If you could move the sensor to an ADC1 pin (32-39), you could keep WiFi on during
dives.** This would simplify the code significantly, but may require hardware changes.

### The Data Recording Requirement

MATE requires **one data packet every 5 seconds** during both profiles, and **7 sequential
packets in range** to prove 30-second holds. With a 5-second interval:

- 7 packets * 5 seconds = 35 seconds (covers the 30-second hold with margin)
- Two profiles, each with descend + hold deep + ascend + hold shallow
- Estimated ~40 total data points across both profiles
- Need at least 20 data points for graphing

### What "Sensor Offset" Means

The depth sensor is mounted on the SIDE of the float, not at the top or bottom:

```
    +---------+  ← TOP (judge measures this for 40cm hold)
    |         |
    |  42.4cm |  ← distance from sensor to top
    |         |
    |    *    |  ← SENSOR position (what the code actually reads)
    |         |
    |  25.0cm |  ← distance from sensor to bottom
    |         |
    +---------+  ← BOTTOM (judge measures this for 2.5m hold)
```

When the judge says "bottom of float is at 2.5m," the sensor reads 2.5 - 0.25 = **2.25m**.
When the judge says "top of float is at 0.4m," the sensor reads 0.4 + 0.424 = **0.824m**.

The code must use these SENSOR target values, not the raw physical depths.
