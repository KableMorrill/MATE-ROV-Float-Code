// ============================================================================
// MATE Float 2026 — Dry Test Build v2
// ============================================================================
//
// PURPOSE: Test the full dive state machine WITHOUT water. Simulated depth
// values are generated internally so the state machine transitions through
// every phase exactly as it would in a real dive. Motors are DISABLED to
// prevent syringe damage.
//
// HOW IT WORKS:
//   - A compile-time flag (DRY_TEST_MODE) swaps the real depth sensor for a
//     simulated one that produces depth values based on the CURRENT STATE
//     and how long the float has been in that state.
//   - The state machine logic is IDENTICAL to the water version. If the dry
//     test completes both profiles correctly, the logic is validated.
//   - Motors do not run. Encoder does not move. All motor calls are no-ops.
//   - Serial output clearly labels all values as SIMULATED.
//
// WHAT TO WATCH FOR IN SERIAL MONITOR:
//   - State transitions happening at the right times
//   - "HOLD" timers counting to 30 seconds without resetting
//   - 7 sequential in-range data points during each hold phase
//   - Two full profiles completing
//   - Data transmission after pressing TRANSMIT
//
// TO SWITCH TO WATER MODE:
//   Set DRY_TEST_MODE to false. All motor, sensor, and encoder functions
//   will use real hardware. No other code changes needed.
//
// ============================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

// ========================= MODE SELECTION =========================
// Set to true for desk testing (fake depth, motors disabled)
// Set to false for water testing (real sensor, real motors)
#define DRY_TEST_MODE false

// ========================= PIN DEFINITIONS =========================
#define DEPTH_PIN 25
#define IN3 18        // L298N IN3 — direction control A
#define IN4 19        // L298N IN4 — direction control B
#define ENB 15        // L298N ENB — PWM speed control
#define ENC1 17       // Encoder channel A
#define ENC2 4        // Encoder channel B
#define LED_DEEP 2    // Blue LED — lit when in deep hold range

// ========================= PWM CONFIGURATION =========================
#define PWM_FREQ 1000        // 1 kHz
#define PWM_RESOLUTION 8     // 8-bit (0-255)

// ========================= MOTOR SPEEDS =========================
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 60
#define APPROACH_ZONE 0.50       // Meters from target to start slowing
#define STOP_EARLY_DISTANCE 0.08 // Meters from target to coast
#define HOLD_CORRECTION_MAX 120  // Max PWM during hold corrections
#define HOLD_CORRECTION_MIN 50   // Min PWM during hold corrections
#define PROACTIVE_ZONE 0.60      // Fraction of tolerance band considered "safe"
                                 // Beyond this (but still in range), apply gentle correction
#define PROACTIVE_SPEED 55       // PWM for proactive nudges (just above min)

// ========================= ENCODER LIMITS =========================
#define PULSES_PER_REV 2600
#define ENCODER_MIN_COUNT (PULSES_PER_REV * -11) // -28600: max syringe extension
#define ENCODER_MAX_COUNT 0                       // Home position
#define ENCODER_HOME_DEADBAND 50                  // +/- pulses for "at home"

// ========================= COMPETITION PARAMETERS =========================
#define TARGET_DEEP 2.5          // Deep hold: bottom of float at this depth
#define TARGET_SHALLOW 0.40      // Shallow hold: top of float at this depth
#define DEPTH_TOLERANCE 0.33     // +/- tolerance
#define DEEP_MIN (TARGET_DEEP - DEPTH_TOLERANCE)       // 2.17m
#define DEEP_MAX (TARGET_DEEP + DEPTH_TOLERANCE)       // 2.83m
#define SHALLOW_MIN (TARGET_SHALLOW - DEPTH_TOLERANCE) // 0.07m
#define SHALLOW_MAX (TARGET_SHALLOW + DEPTH_TOLERANCE) // 0.73m

#define HOLD_DURATION_MS 30000   // 30 seconds
#define DATA_INTERVAL_MS 5000    // Record every 5 seconds
#define NUM_PROFILES 2

// ========================= DATA STORAGE =========================
#define MAX_DATA_POINTS 100      // 2 profiles + margin for hold re-entries
#define DATA_PER_PACKET 10       // Points per ESP-NOW packet

// ========================= SENSOR OFFSETS =========================
// Distance from the depth sensor to the bottom and top of the float.
// These translate between what the sensor reads and what the judge measures.
//
//     +---------+  <- TOP of float
//     |  42.4cm |
//     |    *    |  <- Depth sensor
//     |  25.0cm |
//     +---------+  <- BOTTOM of float
//
#define SENSOR_TO_BOTTOM 0.25    // meters
#define SENSOR_TO_TOP 0.424      // meters

// Sensor readings that correspond to judge's target depths:
//   Deep hold:    bottom at 2.5m  -> sensor reads 2.5 - 0.25 = 2.25m
//   Shallow hold: top at 0.40m   -> sensor reads 0.40 + 0.424 = 0.824m

// ========================= EGADS CORRECTION =========================
#define EGADS_SPECIFIC_GRAVITY 1.000    // *********SET TO 1.025 FOR COMPETITION, CURRENTLY SET TO 1.000 FOR FRESHWATER TESTING********** <-----------
#define EGADS_CORRECTION (1.0 / EGADS_SPECIFIC_GRAVITY)

// ========================= DRY TEST SIMULATION =========================
// How long (ms) simulated depth takes to ramp between states.
// These should roughly match how long the real float takes in water.
// Adjust based on experience — longer = more conservative simulation.
#define SIM_DESCENT_MS 25000   // 25 seconds to descend surface -> 2.5m
#define SIM_ASCENT_MS 20000    // 20 seconds to ascend 2.5m -> 0.4m

// ========================= COMPANY ID =========================
#define COMPANY_NUMBER "PN00"  // *********UPDATE WITH MATE-ASSIGNED COMPANY NUMBER********** <-----------

// ========================= ESP-NOW =========================
// Mission station MAC address (the float sends TO this address)
uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x10, 0x07, 0x34};
esp_now_peer_info_t peerInfo;

// ========================= DATA STRUCTURES =========================
typedef struct struct_message {
    char companyNum[8];
    float time;
    float pressure_kpa;
    float depth_m;
} struct_message;

// State machine
enum ProfileState {
    STATE_IDLE,
    STATE_PRE_DIVE_TX,
    STATE_DESCENDING,
    STATE_HOLD_DEEP,
    STATE_ASCENDING,
    STATE_HOLD_SHALLOW,
    STATE_COMPLETE,
    STATE_RETURNING_HOME,
    STATE_TRANSMITTING
};

// ========================= GLOBAL STATE =========================
struct_message myData[MAX_DATA_POINTS];
struct_message dataPacket[DATA_PER_PACKET];
int dataIndex = 0;
int currentPacket = 0;

ProfileState currentState = STATE_IDLE;
int currentProfile = 0;

unsigned long profileStartTime = 0;   // When the overall recording started
unsigned long stateEntryTime = 0;     // When the current state began
unsigned long holdInRangeStart = 0;   // When the hold timer started
unsigned long lastDataTime = 0;       // Last data recording timestamp

bool motorsMoving = false;
bool descending = true;
bool wifiOn = false;
bool holdInRange = false;
int currentMotorSpeed = 0;

ESP32Encoder buoyEnc;

// Precomputed sensor targets
float sensorTargetDeep;    // What the sensor should read at deep hold
float sensorTargetShallow; // What the sensor should read at shallow hold
float sensorDeepMin, sensorDeepMax;
float sensorShallowMin, sensorShallowMax;

// Simulation state
float simDepthAtStateEntry = 0.0; // Depth when current state began

// ========================= FUNCTION DECLARATIONS =========================
// (Arduino doesn't strictly require these, but they help readability)
float getSimulatedSensorDepth();
float getSensorDepth();
float getPressureFromDepth(float depth);
float getBottomDepth(float sensorDepth);
float getTopDepth(float sensorDepth);
bool isInDeepRange(float sensorDepth);
bool isInShallowRange(float sensorDepth);
void enterState(ProfileState newState);
void recordDataPoint(float sensorDepth);

// ============================================================================
//                                SETUP
// ============================================================================
void setup() {
    Serial.begin(9600);
    delay(500); // Let serial settle

    Serial.println("\n============================================");
    if (DRY_TEST_MODE) {
        Serial.println("  MATE Float 2026 — DRY TEST MODE");
        Serial.println("  Motors DISABLED. Depth is SIMULATED.");
    } else {
        Serial.println("  MATE Float 2026 — WATER MODE");
        Serial.println("  Motors ACTIVE. Depth from REAL sensor.");
    }
    Serial.println("============================================");

    // Motor pin setup — IN3/IN4 for direction (digital), ENB for speed (PWM)
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    if (!DRY_TEST_MODE) {
        ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
        ledcWrite(ENB, 0);
    } else {
        pinMode(ENB, OUTPUT);
        digitalWrite(ENB, LOW);
    }

    // LED setup
    pinMode(LED_DEEP, OUTPUT);
    digitalWrite(LED_DEEP, LOW);

    // Encoder setup
    buoyEnc.attachHalfQuad(ENC1, ENC2);
    buoyEnc.clearCount();

    // Precompute sensor target values
    sensorTargetDeep = TARGET_DEEP - SENSOR_TO_BOTTOM;           // 2.25m
    sensorTargetShallow = TARGET_SHALLOW + SENSOR_TO_TOP;        // 0.824m
    sensorDeepMin = DEEP_MIN - SENSOR_TO_BOTTOM;                 // 1.92m
    sensorDeepMax = DEEP_MAX - SENSOR_TO_BOTTOM;                 // 2.58m
    sensorShallowMin = SHALLOW_MIN + SENSOR_TO_TOP;              // 0.494m
    sensorShallowMax = SHALLOW_MAX + SENSOR_TO_TOP;              // 1.154m

    // Initialize ESP-NOW
    if (setupPeer()) {
        Serial.println("ERROR: ESP-NOW init failed. Check wiring.");
        return;
    }

    // Print configuration
    Serial.println("\nConfiguration:");
    Serial.print("  Company:        "); Serial.println(COMPANY_NUMBER);
    Serial.print("  Float height:   "); Serial.print((SENSOR_TO_BOTTOM + SENSOR_TO_TOP) * 100, 1); Serial.println(" cm");
    Serial.print("  Sensor→bottom:  "); Serial.print(SENSOR_TO_BOTTOM * 100, 1); Serial.println(" cm");
    Serial.print("  Sensor→top:     "); Serial.print(SENSOR_TO_TOP * 100, 1); Serial.println(" cm");

    Serial.println("\nSensor target ranges (for scoring):");
    Serial.print("  Deep hold:    "); Serial.print(sensorDeepMin, 2);
    Serial.print("m - "); Serial.print(sensorDeepMax, 2); Serial.println("m");
    Serial.print("  Deep target:  "); Serial.print(sensorTargetDeep, 2); Serial.println("m");
    Serial.print("  Shallow hold: "); Serial.print(sensorShallowMin, 2);
    Serial.print("m - "); Serial.print(sensorShallowMax, 2); Serial.println("m");
    Serial.print("  Shallow tgt:  "); Serial.print(sensorTargetShallow, 3); Serial.println("m");

    if (DRY_TEST_MODE) {
        Serial.println("\nSimulation timing:");
        Serial.print("  Descent: "); Serial.print(SIM_DESCENT_MS / 1000); Serial.println("s");
        Serial.print("  Ascent:  "); Serial.print(SIM_ASCENT_MS / 1000); Serial.println("s");
        Serial.print("  Hold:    "); Serial.print(HOLD_DURATION_MS / 1000); Serial.println("s");
        Serial.print("  Est. total per profile: ~");
        Serial.print((SIM_DESCENT_MS + HOLD_DURATION_MS + SIM_ASCENT_MS + HOLD_DURATION_MS) / 1000);
        Serial.println("s");
    }

    Serial.println("\nReady. Waiting for START command (D21 on station)...\n");
}

// ============================================================================
//                          DEPTH READING
// ============================================================================

// Real sensor reading (for water mode)
float readRealDepthSensor() {
    float analogReading = analogRead(DEPTH_PIN);
    float voltage = (analogReading / 4095.0) * 3.3;
    float mAmps = (voltage / 150.0) * 1000;
    float depth_raw = (312.5 * (mAmps - 4)) / 1000;
    return depth_raw * EGADS_CORRECTION;
}

// Simulated depth based on current state + time in state
// This is the heart of the dry test — it produces smooth, predictable
// depth ramps that let the state machine exercise every transition.
float getSimulatedSensorDepth() {
    if (currentState == STATE_IDLE || currentState == STATE_PRE_DIVE_TX) {
        return 0.0;
    }
    if (currentState == STATE_COMPLETE || currentState == STATE_RETURNING_HOME
        || currentState == STATE_TRANSMITTING) {
        return 0.0;
    }

    unsigned long timeInState = millis() - stateEntryTime;
    float progress;

    switch (currentState) {
        case STATE_DESCENDING:
            // Ramp from entry depth toward sensorTargetDeep
            progress = constrain(timeInState / (float)SIM_DESCENT_MS, 0.0, 1.0);
            return simDepthAtStateEntry + progress * (sensorTargetDeep - simDepthAtStateEntry);

        case STATE_HOLD_DEEP:
            // Hold at sensorTargetDeep with very slight drift for realism
            // The drift stays well within the scoring range
            return sensorTargetDeep + 0.02 * sin(timeInState / 5000.0);

        case STATE_ASCENDING:
            // Ramp from entry depth toward sensorTargetShallow
            progress = constrain(timeInState / (float)SIM_ASCENT_MS, 0.0, 1.0);
            return simDepthAtStateEntry + progress * (sensorTargetShallow - simDepthAtStateEntry);

        case STATE_HOLD_SHALLOW:
            // Hold at sensorTargetShallow with slight drift
            return sensorTargetShallow + 0.02 * sin(timeInState / 5000.0);

        default:
            return 0.0;
    }
}

// Unified depth reading — returns simulated or real based on mode
float getSensorDepth() {
    if (DRY_TEST_MODE) {
        return getSimulatedSensorDepth();
    } else {
        return readRealDepthSensor();
    }
}

// Convert depth to pressure (for data packets)
float getPressureFromDepth(float depth) {
    return 101.325 + (10.055 * depth); // kPa
}

// Physical depth of float bottom from sensor reading
float getBottomDepth(float sensorDepth) {
    return sensorDepth + SENSOR_TO_BOTTOM;
}

// Physical depth of float top from sensor reading
float getTopDepth(float sensorDepth) {
    return sensorDepth - SENSOR_TO_TOP;
}

// ============================================================================
//                          RANGE CHECKING
// ============================================================================

bool isInDeepRange(float sensorDepth) {
    return (sensorDepth >= sensorDeepMin && sensorDepth <= sensorDeepMax);
}

bool isInShallowRange(float sensorDepth) {
    return (sensorDepth >= sensorShallowMin && sensorDepth <= sensorShallowMax);
}

// ============================================================================
//                          LED INDICATORS
// ============================================================================

void updateLEDs(float sensorDepth) {
    // LED_DEEP lights when in either hold range (only one LED available)
    bool inRange = isInDeepRange(sensorDepth) || isInShallowRange(sensorDepth);
    digitalWrite(LED_DEEP, inRange ? HIGH : LOW);
}

// ============================================================================
//                          MOTOR CONTROL
// ============================================================================
// In DRY_TEST_MODE, these update state variables (so the state machine works)
// but do NOT actually drive the motor.

int calculateApproachSpeed(float distanceToTarget) {
    if (distanceToTarget <= STOP_EARLY_DISTANCE) return 0;
    if (distanceToTarget >= APPROACH_ZONE) return MOTOR_MAX_SPEED;
    float ratio = (distanceToTarget - STOP_EARLY_DISTANCE)
                / (APPROACH_ZONE - STOP_EARLY_DISTANCE);
    return MOTOR_MIN_SPEED + (int)(ratio * (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED));
}

int calculateCorrectionSpeed(float distanceFromRange) {
    if (distanceFromRange <= 0.01) return 0;
    if (distanceFromRange >= APPROACH_ZONE) return HOLD_CORRECTION_MAX;
    float ratio = distanceFromRange / APPROACH_ZONE;
    return HOLD_CORRECTION_MIN + (int)(ratio * (HOLD_CORRECTION_MAX - HOLD_CORRECTION_MIN));
}

void moveFloatDown(int speed) {
    if (!DRY_TEST_MODE) {
        if (buoyEnc.getCount() <= ENCODER_MIN_COUNT) {
            stopFloat();
            Serial.println("SAFETY: Encoder at min limit — descent blocked");
            return;
        }
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(ENB, speed);
    }
    motorsMoving = (speed > 0);
    descending = true;
    currentMotorSpeed = speed;
}

void moveFloatUp(int speed) {
    if (!DRY_TEST_MODE) {
        if (buoyEnc.getCount() >= ENCODER_MAX_COUNT) {
            stopFloat();
            Serial.println("SAFETY: Encoder at max limit — ascent blocked");
            return;
        }
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(ENB, speed);
    }
    motorsMoving = (speed > 0);
    descending = false;
    currentMotorSpeed = speed;
}

void stopFloat() {
    if (!DRY_TEST_MODE) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        ledcWrite(ENB, 0);
    }
    motorsMoving = false;
    currentMotorSpeed = 0;
}

// ============================================================================
//                          ESP-NOW COMMUNICATION
// ============================================================================

bool setupPeer() {
    WiFi.mode(WIFI_STA);
    wifiOn = true;

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return true;
    }
    Serial.println("ESP-NOW initialized");

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return true;
    }
    Serial.println("Peer added");
    return false;
}

void teardownWifi() {
    esp_now_peer_num_t pc;
    esp_now_get_peer_num(&pc);
    if (pc.total_num > 0) {
        esp_now_del_peer(broadcastAddress);
    }
    esp_now_deinit();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    wifiOn = false;
    Serial.println("WiFi disabled for dive");
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX: OK" : "TX: FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    struct_message cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));

    if (cmd.time == 1.0) {
        // ---- START COMMAND ----
        Serial.println("\n>>> START COMMAND RECEIVED <<<");
        currentProfile = 0;
        dataIndex = 0;
        profileStartTime = millis();
        holdInRange = false;

        for (int i = 0; i < MAX_DATA_POINTS; i++) {
            memset(&myData[i], 0, sizeof(struct_message));
        }

        enterState(STATE_PRE_DIVE_TX);
    }
    else if (cmd.time == 2.0) {
        // ---- MANUAL MOTOR CONTROL ----
        if (currentState != STATE_IDLE) {
            Serial.println("WARNING: Manual control ignored — dive in progress");
            return;
        }
        if (cmd.depth_m == -1.0)     moveFloatUp(MOTOR_MAX_SPEED);
        else if (cmd.depth_m == 0.0) stopFloat();
        else if (cmd.depth_m == 1.0) moveFloatDown(MOTOR_MAX_SPEED);
    }
    else if (cmd.time == 3.0) {
        // ---- TRANSMIT COMMAND ----
        Serial.println("\n>>> TRANSMIT COMMAND RECEIVED <<<");
        enterState(STATE_RETURNING_HOME);
    }
}

// ============================================================================
//                          DATA TRANSMISSION
// ============================================================================

void sendPreDivePacket(float sensorDepth) {
    struct_message pkt;
    strncpy(pkt.companyNum, COMPANY_NUMBER, sizeof(pkt.companyNum) - 1);
    pkt.companyNum[sizeof(pkt.companyNum) - 1] = '\0';
    pkt.time = 0.0;
    pkt.pressure_kpa = getPressureFromDepth(sensorDepth);
    pkt.depth_m = sensorDepth;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));

    Serial.print("Pre-dive TX: ");
    if (result == ESP_OK) {
        Serial.print(pkt.companyNum);
        Serial.print(" | "); Serial.print(pkt.time, 1); Serial.print("s");
        Serial.print(" | "); Serial.print(pkt.pressure_kpa, 1); Serial.print("kPa");
        Serial.print(" | "); Serial.print(pkt.depth_m, 3); Serial.println("m");
    } else {
        Serial.println("FAILED");
    }
}

void sendAllData() {
    if (!wifiOn) {
        if (setupPeer()) {
            Serial.println("ERROR: WiFi setup failed for transmission");
            return;
        }
    }

    int totalPackets = (dataIndex + DATA_PER_PACKET - 1) / DATA_PER_PACKET;
    Serial.print("Sending "); Serial.print(dataIndex);
    Serial.print(" points in "); Serial.print(totalPackets); Serial.println(" packets");

    for (int pkt = 0; pkt < totalPackets; pkt++) {
        // Clear packet buffer
        for (int i = 0; i < DATA_PER_PACKET; i++) {
            memset(&dataPacket[i], 0, sizeof(struct_message));
        }

        // Fill with data
        int startIdx = pkt * DATA_PER_PACKET;
        int count = 0;
        for (int i = 0; i < DATA_PER_PACKET && (startIdx + i) < dataIndex; i++) {
            dataPacket[i] = myData[startIdx + i];
            count++;
        }

        esp_err_t result = esp_now_send(broadcastAddress,
                                         (uint8_t *)&dataPacket, sizeof(dataPacket));

        if (result == ESP_OK) {
            Serial.print("  Packet "); Serial.print(pkt + 1);
            Serial.print("/"); Serial.print(totalPackets);
            Serial.print(" ("); Serial.print(count); Serial.println(" pts) OK");
        } else {
            Serial.print("  Packet "); Serial.print(pkt + 1);
            Serial.println(" FAILED — retrying...");
            delay(50);
            pkt--; // Retry this packet
            continue;
        }

        if (pkt < totalPackets - 1) {
            delay(100); // Brief pause between packets
        }
    }

    Serial.println("All data transmitted.");
}

// ============================================================================
//                          DATA RECORDING
// ============================================================================

void recordDataPoint(float sensorDepth) {
    if (dataIndex >= MAX_DATA_POINTS) {
        Serial.println("WARNING: Data buffer full!");
        return;
    }

    float elapsed = (millis() - profileStartTime) / 1000.0;

    strncpy(myData[dataIndex].companyNum, COMPANY_NUMBER,
            sizeof(myData[dataIndex].companyNum) - 1);
    myData[dataIndex].companyNum[sizeof(myData[dataIndex].companyNum) - 1] = '\0';
    myData[dataIndex].time = elapsed;
    myData[dataIndex].pressure_kpa = getPressureFromDepth(sensorDepth);
    myData[dataIndex].depth_m = sensorDepth;

    float bottom = getBottomDepth(sensorDepth);
    float top = getTopDepth(sensorDepth);

    Serial.print("  DATA["); Serial.print(dataIndex); Serial.print("] ");
    Serial.print(elapsed, 1); Serial.print("s ");
    Serial.print("sensor="); Serial.print(sensorDepth, 3); Serial.print("m ");
    Serial.print("bot="); Serial.print(bottom, 3); Serial.print("m ");
    Serial.print("top="); Serial.print(top, 3); Serial.print("m ");
    if (DRY_TEST_MODE) Serial.print("[SIM]");
    Serial.println();

    dataIndex++;
}

// ============================================================================
//                          STATE TRANSITIONS
// ============================================================================
// Centralized state entry — sets timers and logs the transition.
// simDepthAtStateEntry captures the current depth so ramps start smoothly.

void enterState(ProfileState newState) {
    float currentDepth = getSensorDepth();
    simDepthAtStateEntry = currentDepth;
    stateEntryTime = millis();
    currentState = newState;

    // Log the transition
    switch (newState) {
        case STATE_IDLE:
            Serial.println("[STATE] -> IDLE"); break;
        case STATE_PRE_DIVE_TX:
            Serial.println("[STATE] -> PRE_DIVE_TX"); break;
        case STATE_DESCENDING:
            Serial.print("[STATE] -> DESCENDING (profile ");
            Serial.print(currentProfile + 1); Serial.println(")"); break;
        case STATE_HOLD_DEEP:
            Serial.println("[STATE] -> HOLD_DEEP (30s at 2.5m)"); break;
        case STATE_ASCENDING:
            Serial.println("[STATE] -> ASCENDING (toward 40cm)"); break;
        case STATE_HOLD_SHALLOW:
            Serial.println("[STATE] -> HOLD_SHALLOW (30s at 40cm)"); break;
        case STATE_COMPLETE:
            Serial.println("[STATE] -> COMPLETE — ready for recovery");
            // Re-enable WiFi so we can receive the TRANSMIT command
            if (!wifiOn) {
                if (setupPeer()) {
                    Serial.println("WARNING: WiFi re-init failed — press TRANSMIT to retry");
                } else {
                    Serial.println("WiFi re-enabled. Waiting for TRANSMIT command (D22)...");
                }
            }
            break;
        case STATE_RETURNING_HOME:
            Serial.println("[STATE] -> RETURNING_HOME"); break;
        case STATE_TRANSMITTING:
            Serial.println("[STATE] -> TRANSMITTING"); break;
    }
}

// ============================================================================
//                              MAIN LOOP
// ============================================================================

void loop() {
    static unsigned long lastDebugPrint = 0;
    unsigned long now = millis();

    // ---- Get current depth (simulated or real) ----
    float sensorDepth = getSensorDepth();
    float bottomDepth = getBottomDepth(sensorDepth);
    float topDepth = getTopDepth(sensorDepth);

    // ---- Update LEDs ----
    updateLEDs(sensorDepth);

    // ---- Record data every 5s during active dive states ----
    if (currentState >= STATE_DESCENDING && currentState <= STATE_HOLD_SHALLOW) {
        if (now - lastDataTime >= DATA_INTERVAL_MS) {
            lastDataTime = now;
            recordDataPoint(sensorDepth);
        }
    }

    // ---- State machine ----
    switch (currentState) {

        case STATE_IDLE:
            // Nothing to do — waiting for START command
            break;

        case STATE_PRE_DIVE_TX: {
            // Send one data packet to prove communication, then start diving
            Serial.println("\n--- Pre-dive transmission ---");
            sendPreDivePacket(sensorDepth);
            delay(500); // Let packet go out

            // Tear down WiFi for clean ADC reads
            teardownWifi();

            // Record first data point
            lastDataTime = now;
            recordDataPoint(sensorDepth);

            // Begin first descent
            Serial.print("\n=== PROFILE "); Serial.print(currentProfile + 1);
            Serial.println(" START ===");
            enterState(STATE_DESCENDING);
            moveFloatDown(MOTOR_MAX_SPEED);
            break;
        }

        case STATE_DESCENDING: {
            float distToTarget = sensorTargetDeep - sensorDepth;

            // Drive motor with proportional speed (no-op in dry test)
            if (distToTarget > 0) {
                int speed = calculateApproachSpeed(distToTarget);
                if (speed > 0) {
                    moveFloatDown(speed);
                } else {
                    stopFloat();
                }
            }

            // Check if we've entered the valid deep range
            if (isInDeepRange(sensorDepth)) {
                stopFloat();
                holdInRange = true;
                holdInRangeStart = now;
                lastDataTime = now;  // Sync data interval to hold entry
                enterState(STATE_HOLD_DEEP);
                recordDataPoint(sensorDepth);  // 1st of 7 hold readings
                Serial.print("  Sensor: "); Serial.print(sensorDepth, 3);
                Serial.print("m  Bottom: "); Serial.print(bottomDepth, 3);
                Serial.println("m");
            }
            break;
        }

        case STATE_HOLD_DEEP: {
            if (isInDeepRange(sensorDepth)) {
                if (!holdInRange) {
                    // Re-entered range — restart the 30s timer
                    holdInRange = true;
                    holdInRangeStart = now;
                    lastDataTime = now;  // Re-sync data interval
                    stopFloat();
                    recordDataPoint(sensorDepth);  // 1st reading of new hold attempt
                    Serial.println("  Entered deep range — hold timer RESET");
                }

                // Proactive correction: nudge back toward center before leaving range
                float deepSafeMin = sensorTargetDeep - (DEPTH_TOLERANCE * PROACTIVE_ZONE);
                float deepSafeMax = sensorTargetDeep + (DEPTH_TOLERANCE * PROACTIVE_ZONE);
                if (sensorDepth < deepSafeMin) {
                    moveFloatDown(PROACTIVE_SPEED);
                } else if (sensorDepth > deepSafeMax) {
                    moveFloatUp(PROACTIVE_SPEED);
                } else {
                    stopFloat();
                }

                // Check if 30 seconds have elapsed in range
                if (now - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 2.5m COMPLETE <<<");
                    holdInRange = false;
                    enterState(STATE_ASCENDING);
                    moveFloatUp(MOTOR_MAX_SPEED);
                }
            } else {
                // Drifted out of range
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("  WARNING: Drifted out! Sensor: ");
                    Serial.print(sensorDepth, 3); Serial.print("m  Range: ");
                    Serial.print(sensorDeepMin, 2); Serial.print("-");
                    Serial.print(sensorDeepMax, 2); Serial.println("m");
                    Serial.println("  Hold timer RESET — need full 30s in range");
                }

                // Correction (no-op in dry test, but logic is exercised)
                if (sensorDepth < sensorDeepMin) {
                    int speed = calculateCorrectionSpeed(sensorDeepMin - sensorDepth);
                    if (speed > 0) moveFloatDown(speed);
                } else if (sensorDepth > sensorDeepMax) {
                    int speed = calculateCorrectionSpeed(sensorDepth - sensorDeepMax);
                    if (speed > 0) moveFloatUp(speed);
                }
            }
            break;
        }

        case STATE_ASCENDING: {
            float distToTarget = sensorDepth - sensorTargetShallow;

            if (distToTarget > 0) {
                int speed = calculateApproachSpeed(distToTarget);
                if (speed > 0) {
                    moveFloatUp(speed);
                } else {
                    stopFloat();
                }
            }

            // Check if we've entered shallow range
            if (isInShallowRange(sensorDepth)) {
                stopFloat();
                holdInRange = true;
                holdInRangeStart = now;
                lastDataTime = now;  // Sync data interval to hold entry
                enterState(STATE_HOLD_SHALLOW);
                recordDataPoint(sensorDepth);  // 1st of 7 hold readings
                Serial.print("  Sensor: "); Serial.print(sensorDepth, 3);
                Serial.print("m  Top: "); Serial.print(topDepth, 3);
                Serial.println("m");
            }
            // Surface breach prevention (real mode — doesn't apply in dry sim)
            else if (!DRY_TEST_MODE && sensorDepth < sensorShallowMin) {
                stopFloat();
                Serial.print("  WARNING: Above shallow range! Top at ");
                Serial.print(topDepth, 3); Serial.println("m");
                int speed = calculateCorrectionSpeed(sensorShallowMin - sensorDepth);
                if (speed > 0) moveFloatDown(speed);
            }
            break;
        }

        case STATE_HOLD_SHALLOW: {
            if (isInShallowRange(sensorDepth)) {
                if (!holdInRange) {
                    holdInRange = true;
                    holdInRangeStart = now;
                    lastDataTime = now;  // Re-sync data interval
                    stopFloat();
                    recordDataPoint(sensorDepth);  // 1st reading of new hold attempt
                    Serial.println("  Entered shallow range — hold timer RESET");
                }

                // Proactive correction: nudge back toward center before leaving range
                float shallowSafeMin = sensorTargetShallow - (DEPTH_TOLERANCE * PROACTIVE_ZONE);
                float shallowSafeMax = sensorTargetShallow + (DEPTH_TOLERANCE * PROACTIVE_ZONE);
                if (sensorDepth < shallowSafeMin) {
                    moveFloatDown(PROACTIVE_SPEED);
                } else if (sensorDepth > shallowSafeMax) {
                    moveFloatUp(PROACTIVE_SPEED);
                } else {
                    stopFloat();
                }

                if (now - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 40cm COMPLETE <<<");
                    Serial.print("=== PROFILE "); Serial.print(currentProfile + 1);
                    Serial.println(" COMPLETE ===\n");

                    currentProfile++;
                    holdInRange = false;

                    if (currentProfile < NUM_PROFILES) {
                        // Start next profile
                        Serial.print("=== PROFILE "); Serial.print(currentProfile + 1);
                        Serial.println(" START ===");
                        enterState(STATE_DESCENDING);
                        moveFloatDown(MOTOR_MAX_SPEED);
                    } else {
                        // All profiles done
                        stopFloat();
                        enterState(STATE_COMPLETE);
                        Serial.println("========================================");
                        Serial.println("ALL PROFILES COMPLETE");
                        Serial.print("Total data points: "); Serial.println(dataIndex);
                        Serial.println("Press TRANSMIT (D22) to send data.");
                        Serial.println("========================================");
                    }
                }
            } else {
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("  WARNING: Drifted out! Sensor: ");
                    Serial.print(sensorDepth, 3); Serial.print("m  Range: ");
                    Serial.print(sensorShallowMin, 2); Serial.print("-");
                    Serial.print(sensorShallowMax, 2); Serial.println("m");
                    Serial.println("  Hold timer RESET — need full 30s in range");
                }

                if (sensorDepth > sensorShallowMax) {
                    int speed = calculateCorrectionSpeed(sensorDepth - sensorShallowMax);
                    if (speed > 0) moveFloatUp(speed);
                } else if (sensorDepth < sensorShallowMin) {
                    int speed = calculateCorrectionSpeed(sensorShallowMin - sensorDepth);
                    if (speed > 0) moveFloatDown(speed);
                }
            }
            break;
        }

        case STATE_COMPLETE: {
            // Flash LED to signal completion
            digitalWrite(LED_DEEP, (now / 500) % 2 == 0 ? HIGH : LOW);
            break;
        }

        case STATE_RETURNING_HOME: {
            if (DRY_TEST_MODE) {
                // No motor to return — skip straight to transmit
                Serial.println("  (Dry test: skipping motor return)");
                enterState(STATE_TRANSMITTING);
                break;
            }

            long encCount = buoyEnc.getCount();
            long absCount = abs(encCount);

            if (absCount <= ENCODER_HOME_DEADBAND) {
                stopFloat();
                Serial.print("  Home reached (encoder: "); Serial.print(encCount);
                Serial.println(")");
                enterState(STATE_TRANSMITTING);
            } else {
                int speed;
                if (absCount > PULSES_PER_REV * 2) speed = MOTOR_MAX_SPEED;
                else if (absCount > PULSES_PER_REV / 2) speed = HOLD_CORRECTION_MAX;
                else speed = HOLD_CORRECTION_MIN;

                if (encCount < 0) moveFloatUp(speed);
                else moveFloatDown(speed);

                static unsigned long lastHomePrint = 0;
                if (now - lastHomePrint >= 1000) {
                    lastHomePrint = now;
                    Serial.print("  Returning: enc="); Serial.print(encCount);
                    Serial.print(" speed="); Serial.println(speed);
                }
            }
            break;
        }

        case STATE_TRANSMITTING: {
            Serial.println("\n--- Transmitting all profile data ---");
            sendAllData();
            enterState(STATE_IDLE);
            Serial.println("Transmission complete. Returning to IDLE.\n");
            break;
        }
    }

    // ---- Debug output every second during active states ----
    if (now - lastDebugPrint >= 1000
        && currentState >= STATE_DESCENDING
        && currentState <= STATE_HOLD_SHALLOW) {

        lastDebugPrint = now;

        unsigned long timeInState = (now - stateEntryTime) / 1000;

        Serial.print("[P"); Serial.print(currentProfile + 1); Serial.print("] ");

        // State name
        switch (currentState) {
            case STATE_DESCENDING:   Serial.print("DESC  "); break;
            case STATE_HOLD_DEEP:    Serial.print("HDEEP "); break;
            case STATE_ASCENDING:    Serial.print("ASC   "); break;
            case STATE_HOLD_SHALLOW: Serial.print("HSHLO "); break;
            default: break;
        }

        Serial.print("t="); Serial.print(timeInState); Serial.print("s ");
        Serial.print("s="); Serial.print(sensorDepth, 3); Serial.print("m ");
        Serial.print("bot="); Serial.print(bottomDepth, 2); Serial.print("m ");
        Serial.print("top="); Serial.print(topDepth, 2); Serial.print("m ");

        if (holdInRange) {
            unsigned long holdSec = (now - holdInRangeStart) / 1000;
            Serial.print("HOLD:"); Serial.print(holdSec); Serial.print("/30s ");
        }

        if (isInDeepRange(sensorDepth))    Serial.print("[DEEP OK]");
        if (isInShallowRange(sensorDepth)) Serial.print("[SHALLOW OK]");
        if (DRY_TEST_MODE) Serial.print(" [SIM]");
        Serial.println();
    }
}
