#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

// ============== PIN DEFINITIONS ==============
#define DEPTH_PIN 25
#define IN1 18  // buoyancy motor pins
#define IN2 19
#define ENC1 17
#define ENC2 4
#define LED_DEEP 2      // Blue LED - indicates at 2.5m range
#define LED_SHALLOW 15  // Green LED - indicates at 40cm range

// ============== MOTOR/ENCODER SETTINGS ==============
#define PULSES_PER_REV 2600
#define TARGET_DIVE_PULSES (PULSES_PER_REV * -8)
#define TARGET_SURFACE_PULSES 0

// ============== COMPETITION PARAMETERS ==============
// Depth targets (in meters)
#define TARGET_DEEP 2.5       // Deep hold depth (bottom of float)
#define TARGET_SHALLOW 0.40   // Shallow hold depth (top of float)
#define DEPTH_TOLERANCE 0.33  // +/- 33cm tolerance

// Calculated ranges for scoring
#define DEEP_MIN (TARGET_DEEP - DEPTH_TOLERANCE)    // 2.17m
#define DEEP_MAX (TARGET_DEEP + DEPTH_TOLERANCE)    // 2.83m
#define SHALLOW_MIN (TARGET_SHALLOW - DEPTH_TOLERANCE)  // 0.07m
#define SHALLOW_MAX (TARGET_SHALLOW + DEPTH_TOLERANCE)  // 0.73m

// Timing
#define HOLD_DURATION_MS 30000  // 30 seconds hold at each depth
#define DATA_INTERVAL_MS 5000   // Record data every 5 seconds
#define NUM_PROFILES 2          // Number of vertical profiles

// Data storage
#define MAX_DATA_POINTS 60
#define DATA_PER_PACKET 10

// ============== EGADS ICE TANK CORRECTION ==============
// EGADS solution specific gravity is ~1.025
// Pressure sensors calibrated for freshwater (SG 1.0) will read ~2.5% low
#define EGADS_SPECIFIC_GRAVITY 1.025
#define EGADS_CORRECTION_FACTOR (1.0 / EGADS_SPECIFIC_GRAVITY)

// ******************************************************************************
// *                     DEPTH SENSOR CALIBRATION OFFSETS                       *
// ******************************************************************************
//
// DEPTH_ZERO_OFFSET: Corrects for sensor reading negative at surface
// -------------------------------------------------------------------------
// Your sensor currently reads -0.02m to -0.26m at the surface (should be 0).
// This offset is ADDED to the raw depth reading to bring surface to ~0.
//
// Current setting: +0.12m (+12cm) based on average surface error
//
// To calibrate: Place float on surface, read sensor value, negate it.
// Example: If sensor reads -0.15m at surface, set DEPTH_ZERO_OFFSET to 0.15
//
#define DEPTH_ZERO_OFFSET 0.12  // meters (add 12cm to all readings)

// ******************************************************************************
// *                    PHYSICAL FLOAT DIMENSION OFFSETS                        *
// ******************************************************************************
//
// The depth sensor is mounted on the SIDE of the float, not at the top or bottom.
// These offsets account for the physical distance from the sensor to each end.
//
//     +---------+  <-- TOP of float
//     |         |
//     |    *    |  <-- Depth sensor location (side-mounted)
//     |         |
//     +---------+  <-- BOTTOM of float
//
// SENSOR_TO_BOTTOM_OFFSET: Distance from sensor to BOTTOM of float
// -------------------------------------------------------------------------
// Used for 2.5m hold (judges measure from bottom of float)
//
// How to measure: With float vertical, measure from sensor to bottom edge.
//
// Example: If sensor is 25cm above the bottom, set to 0.25
//          When sensor reads 2.25m, bottom is at 2.25 + 0.25 = 2.50m
//
// Scoring range for sensor (with offset applied):
//   Bottom must be 2.17m - 2.83m
//   So sensor must read: (2.17 - offset) to (2.83 - offset)
//   With 0.25m offset: sensor reads 1.92m - 2.58m
//
#define SENSOR_TO_BOTTOM_OFFSET 0.00  // TODO: Measure and update (in meters)

// SENSOR_TO_TOP_OFFSET: Distance from sensor to TOP of float
// -------------------------------------------------------------------------
// Used for 40cm hold (judges measure from top of float)
//
// How to measure: With float vertical, measure from sensor to top edge.
//
// Example: If sensor is 15cm below the top, set to 0.15
//          When sensor reads 0.55m, top is at 0.55 - 0.15 = 0.40m
//
// Scoring range for sensor (with offset applied):
//   Top must be 0.07m - 0.73m
//   So sensor must read: (0.07 + offset) to (0.73 + offset)
//   With 0.15m offset: sensor reads 0.22m - 0.88m
//
#define SENSOR_TO_TOP_OFFSET 0.00  // TODO: Measure and update (in meters)

// ******************************************************************************
// *                         OFFSET SUMMARY FOR JUDGES                          *
// ******************************************************************************
//
// Before competition, communicate these adjusted ranges to the station judge:
//
// For 2.5m hold (bottom of float):
//   Standard range: 2.17m - 2.83m (for bottom of float)
//   Sensor position: [SENSOR_TO_BOTTOM_OFFSET] meters above bottom
//   Adjusted sensor range: [2.17 - offset] to [2.83 - offset]
//
// For 40cm hold (top of float):
//   Standard range: 0.07m - 0.73m (for top of float)
//   Sensor position: [SENSOR_TO_TOP_OFFSET] meters below top
//   Adjusted sensor range: [0.07 + offset] to [0.73 + offset]
//
// ******************************************************************************

// ============== COMPANY IDENTIFIER ==============
#define COMPANY_NUMBER "PN00"  // TODO: Update with MATE-assigned company number

// ============== ESP-NOW CONFIGURATION ==============
uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x10, 0x07, 0x34};

esp_now_peer_info_t peerInfo;

// ============== DATA STRUCTURES ==============
typedef struct struct_message {
    char companyNum[8];   // Company identifier
    float time;           // Time in seconds (float time since start)
    float pressure_kpa;   // Pressure in kilopascals
    float depth_m;        // Depth in meters
} struct_message;

// State machine for profile execution
enum ProfileState {
    STATE_IDLE,
    STATE_PRE_DIVE_TX,
    STATE_DESCENDING,
    STATE_HOLD_DEEP,
    STATE_ASCENDING,
    STATE_HOLD_SHALLOW,
    STATE_BETWEEN_PROFILES,
    STATE_COMPLETE,
    STATE_TRANSMITTING
};

// ============== GLOBAL VARIABLES ==============
struct_message myData[MAX_DATA_POINTS];
struct_message dataPacket[DATA_PER_PACKET];
int dataIndex = 0;
int currentPacket = 0;

ProfileState currentState = STATE_IDLE;
int currentProfile = 0;

unsigned long profileStartTime = 0;
unsigned long holdStartTime = 0;
unsigned long lastDataTime = 0;
unsigned long stateStartTime = 0;

bool motorsMoving = false;
bool descending = true;
bool wifiOn = false;
bool holdInRange = false;
unsigned long holdInRangeStart = 0;

ESP32Encoder buoyEnc;

// ============== FUNCTION DECLARATIONS ==============
void setup();
void loop();
bool setupPeer();
void teardownWifi();
void sendPreDivePacket();
void sendAllData();
void sendDataPacket();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void moveFloatDown();
void moveFloatUp();
void stopFloat();
float readDepthSensorRaw();
float readDepthSensor();
float readPressureSensor();
void recordDataPoint();
void updateLEDs(float depth);
bool isInDeepRange(float sensorDepth);
bool isInShallowRange(float sensorDepth);
float getBottomDepth(float sensorDepth);
float getTopDepth(float sensorDepth);
void printOffsetConfiguration();

// ============== SETUP ==============
void setup() {
    Serial.begin(9600);
    Serial.println("\n==========================================");
    Serial.println("MATE Float 2026 - TEST VERSION 1");
    Serial.println("Ice Tank / Offset Calibration Build");
    Serial.println("==========================================");

    // Motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // LED indicators
    pinMode(LED_DEEP, OUTPUT);
    pinMode(LED_SHALLOW, OUTPUT);
    digitalWrite(LED_DEEP, LOW);
    digitalWrite(LED_SHALLOW, LOW);

    // Encoder setup
    buoyEnc.attachHalfQuad(ENC1, ENC2);
    buoyEnc.clearCount();

    // Initialize WiFi/ESP-NOW
    if (setupPeer()) {
        Serial.println("ERROR: Failed to initialize ESP-NOW");
        return;
    }

    // Print detailed configuration
    printOffsetConfiguration();

    Serial.println("\nReady. Waiting for start command...");
    Serial.println("(Send time=1.0 to begin profile sequence)");
}

// ============== PRINT OFFSET CONFIGURATION ==============
void printOffsetConfiguration() {
    float deepSensorMin = DEEP_MIN - SENSOR_TO_BOTTOM_OFFSET;
    float deepSensorMax = DEEP_MAX - SENSOR_TO_BOTTOM_OFFSET;
    float shallowSensorMin = SHALLOW_MIN + SENSOR_TO_TOP_OFFSET;
    float shallowSensorMax = SHALLOW_MAX + SENSOR_TO_TOP_OFFSET;

    Serial.println("\n--- CALIBRATION CONFIGURATION ---");
    Serial.println("\nDepth Zero Offset:");
    Serial.print("  Raw surface correction: +");
    Serial.print(DEPTH_ZERO_OFFSET * 100, 1); Serial.println(" cm");

    Serial.println("\nPhysical Float Offsets:");
    Serial.print("  Sensor to BOTTOM: ");
    Serial.print(SENSOR_TO_BOTTOM_OFFSET * 100, 1); Serial.println(" cm");
    Serial.print("  Sensor to TOP:    ");
    Serial.print(SENSOR_TO_TOP_OFFSET * 100, 1); Serial.println(" cm");

    Serial.println("\nEGADS Correction:");
    Serial.print("  Specific gravity: "); Serial.println(EGADS_SPECIFIC_GRAVITY);
    Serial.print("  Correction factor: "); Serial.println(EGADS_CORRECTION_FACTOR, 4);

    Serial.println("\n--- SCORING RANGES ---");

    Serial.println("\n2.5m Hold (bottom of float):");
    Serial.print("  Judge measures bottom: ");
    Serial.print(DEEP_MIN, 2); Serial.print("m - ");
    Serial.print(DEEP_MAX, 2); Serial.println("m");
    Serial.print("  Sensor must read:      ");
    Serial.print(deepSensorMin, 2); Serial.print("m - ");
    Serial.print(deepSensorMax, 2); Serial.println("m");

    Serial.println("\n40cm Hold (top of float):");
    Serial.print("  Judge measures top: ");
    Serial.print(SHALLOW_MIN, 2); Serial.print("m - ");
    Serial.print(SHALLOW_MAX, 2); Serial.println("m");
    Serial.print("  Sensor must read:   ");
    Serial.print(shallowSensorMin, 2); Serial.print("m - ");
    Serial.print(shallowSensorMax, 2); Serial.println("m");

    Serial.println("\n--- CURRENT SENSOR READING ---");
    float rawDepth = readDepthSensorRaw();
    float correctedDepth = readDepthSensor();
    Serial.print("  Raw depth:       "); Serial.print(rawDepth, 3); Serial.println("m");
    Serial.print("  Corrected depth: "); Serial.print(correctedDepth, 3); Serial.println("m");
    Serial.print("  Bottom position: "); Serial.print(getBottomDepth(correctedDepth), 3); Serial.println("m");
    Serial.print("  Top position:    "); Serial.print(getTopDepth(correctedDepth), 3); Serial.println("m");

    // Warnings for unconfigured offsets
    Serial.println("\n--- CONFIGURATION WARNINGS ---");
    bool hasWarnings = false;

    if (SENSOR_TO_BOTTOM_OFFSET == 0.0) {
        Serial.println("  WARNING: SENSOR_TO_BOTTOM_OFFSET is 0 - measure and update!");
        hasWarnings = true;
    }
    if (SENSOR_TO_TOP_OFFSET == 0.0) {
        Serial.println("  WARNING: SENSOR_TO_TOP_OFFSET is 0 - measure and update!");
        hasWarnings = true;
    }
    if (strcmp(COMPANY_NUMBER, "PN00") == 0) {
        Serial.println("  WARNING: COMPANY_NUMBER not set - update before competition!");
        hasWarnings = true;
    }
    if (!hasWarnings) {
        Serial.println("  All offsets configured.");
    }

    Serial.println("\n----------------------------------");
}

// ============== ESP-NOW SETUP ==============
bool setupPeer() {
    WiFi.mode(WIFI_STA);
    wifiOn = true;

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return true;
    }
    Serial.println("ESP-NOW initialized successfully");

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return true;
    }
    Serial.println("Peer added successfully");

    return false;
}

// ============== ESP-NOW TEARDOWN ==============
void teardownWifi() {
    esp_now_peer_num_t peer_count;
    esp_now_get_peer_num(&peer_count);

    if (peer_count.total_num > 0) {
        if (esp_now_del_peer(broadcastAddress) == ESP_OK) {
            Serial.println("Peer removed successfully");
        }
    }

    esp_now_deinit();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    wifiOn = false;
    Serial.println("WiFi disabled for dive");
}

// ============== PRE-DIVE TRANSMISSION ==============
void sendPreDivePacket() {
    if (!wifiOn) {
        if (setupPeer()) {
            Serial.println("ERROR: Could not setup WiFi for pre-dive TX");
            return;
        }
    }

    struct_message preDivePacket;
    strncpy(preDivePacket.companyNum, COMPANY_NUMBER, sizeof(preDivePacket.companyNum) - 1);
    preDivePacket.companyNum[sizeof(preDivePacket.companyNum) - 1] = '\0';
    preDivePacket.time = 0.0;
    preDivePacket.pressure_kpa = readPressureSensor();
    preDivePacket.depth_m = readDepthSensor();

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&preDivePacket, sizeof(preDivePacket));

    if (result == ESP_OK) {
        Serial.println("Pre-dive packet sent successfully");
        Serial.print("  Company: "); Serial.println(preDivePacket.companyNum);
        Serial.print("  Time: "); Serial.print(preDivePacket.time); Serial.println("s");
        Serial.print("  Pressure: "); Serial.print(preDivePacket.pressure_kpa); Serial.println(" kPa");
        Serial.print("  Depth: "); Serial.print(preDivePacket.depth_m); Serial.println(" m");
    } else {
        Serial.println("ERROR: Failed to send pre-dive packet");
    }
}

// ============== POST-RECOVERY DATA TRANSMISSION ==============
void sendAllData() {
    if (!wifiOn) {
        if (setupPeer()) {
            Serial.println("ERROR: Could not setup WiFi for data TX");
            return;
        }
    }

    int totalPackets = (dataIndex + DATA_PER_PACKET - 1) / DATA_PER_PACKET;
    Serial.print("Transmitting "); Serial.print(dataIndex); Serial.print(" data points in ");
    Serial.print(totalPackets); Serial.println(" packets");

    currentPacket = 0;
    sendDataPacket();
}

void sendDataPacket() {
    if (currentPacket * DATA_PER_PACKET >= dataIndex) {
        Serial.println("All data packets transmitted");
        return;
    }

    // Clear packet
    for (int i = 0; i < DATA_PER_PACKET; i++) {
        memset(&dataPacket[i], 0, sizeof(struct_message));
    }

    // Fill packet
    int startIdx = currentPacket * DATA_PER_PACKET;
    int count = 0;
    for (int i = 0; i < DATA_PER_PACKET && (startIdx + i) < dataIndex; i++) {
        dataPacket[i] = myData[startIdx + i];
        count++;
    }

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dataPacket, sizeof(dataPacket));

    if (result == ESP_OK) {
        Serial.print("Packet "); Serial.print(currentPacket + 1);
        Serial.print(" sent ("); Serial.print(count); Serial.println(" points)");
        currentPacket++;

        if (currentPacket * DATA_PER_PACKET < dataIndex) {
            delay(100);
            sendDataPacket();
        }
    } else {
        Serial.println("ERROR: Packet send failed, retrying...");
        delay(50);
        sendDataPacket();
    }
}

// ============== ESP-NOW CALLBACKS ==============
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX Success" : "TX Failed");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    struct_message packetData;
    memcpy(&packetData, incomingData, sizeof(packetData));

    // Command: time=1.0 -> Start dive sequence
    if (packetData.time == 1.0) {
        Serial.println("\n>>> START COMMAND RECEIVED <<<");

        currentState = STATE_PRE_DIVE_TX;
        currentProfile = 0;
        dataIndex = 0;
        profileStartTime = millis();
        lastDataTime = 0;
        holdInRange = false;
        buoyEnc.clearCount();

        for (int i = 0; i < MAX_DATA_POINTS; i++) {
            memset(&myData[i], 0, sizeof(struct_message));
        }

        Serial.println("Profile sequence initiated");
    }
    // Command: time=2.0 -> Manual motor control
    else if (packetData.time == 2.0) {
        if (packetData.depth_m == -1.0) {
            moveFloatUp();
        } else if (packetData.depth_m == 0.0) {
            stopFloat();
        } else if (packetData.depth_m == 1.0) {
            moveFloatDown();
        }
    }
    // Command: time=3.0 -> Trigger data transmission (after recovery)
    else if (packetData.time == 3.0) {
        Serial.println("\n>>> TRANSMIT COMMAND RECEIVED <<<");
        currentState = STATE_TRANSMITTING;
    }
    // Command: time=4.0 -> Print current calibration/sensor status
    else if (packetData.time == 4.0) {
        Serial.println("\n>>> CALIBRATION STATUS REQUEST <<<");
        printOffsetConfiguration();
    }
}

// ============== MOTOR CONTROL ==============
void moveFloatDown() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    motorsMoving = true;
    descending = true;
    Serial.println("Motor: DESCENDING");
}

void moveFloatUp() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    motorsMoving = true;
    descending = false;
    Serial.println("Motor: ASCENDING");
}

void stopFloat() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    motorsMoving = false;
    Serial.println("Motor: STOPPED");
}

// ============== SENSOR READINGS ==============

// Read raw depth (no corrections) - for calibration display
float readDepthSensorRaw() {
    float analogReading = analogRead(DEPTH_PIN);
    float voltage = (analogReading / 4095.0) * 3.3;
    float mAmps = (voltage / 150.0) * 1000;
    float depth_raw = (312.5 * (mAmps - 4)) / 1000;
    return depth_raw;
}

// Read depth with all corrections applied
float readDepthSensor() {
    float depth_raw = readDepthSensorRaw();

    // Step 1: Apply zero offset correction (sensor reads negative at surface)
    float depth_zeroed = depth_raw + DEPTH_ZERO_OFFSET;

    // Step 2: Apply EGADS correction for ice tank density
    float depth_corrected = depth_zeroed * EGADS_CORRECTION_FACTOR;

    return depth_corrected;
}

// Read pressure in kilopascals (calculated from corrected depth)
float readPressureSensor() {
    float depth = readDepthSensor();
    // P = rho * g * h, for EGADS solution (rho = 1025 kg/m^3)
    // P (kPa) = 10.055 * depth + atmospheric (101.325 kPa)
    float pressure_kpa = 101.325 + (10.055 * depth);
    return pressure_kpa;
}

// Get depth of float BOTTOM from sensor reading
// Used for 2.5m hold - judges measure from bottom of float
float getBottomDepth(float sensorDepth) {
    return sensorDepth + SENSOR_TO_BOTTOM_OFFSET;
}

// Get depth of float TOP from sensor reading
// Used for 40cm hold - judges measure from top of float
float getTopDepth(float sensorDepth) {
    return sensorDepth - SENSOR_TO_TOP_OFFSET;
}

// ============== DEPTH RANGE CHECKING ==============

// Check if BOTTOM of float is in deep range (2.5m +/- 33cm)
bool isInDeepRange(float sensorDepth) {
    float bottomDepth = getBottomDepth(sensorDepth);
    return (bottomDepth >= DEEP_MIN && bottomDepth <= DEEP_MAX);
}

// Check if TOP of float is in shallow range (40cm +/- 33cm)
bool isInShallowRange(float sensorDepth) {
    float topDepth = getTopDepth(sensorDepth);
    return (topDepth >= SHALLOW_MIN && topDepth <= SHALLOW_MAX);
}

// ============== LED INDICATORS ==============
void updateLEDs(float sensorDepth) {
    if (isInDeepRange(sensorDepth)) {
        digitalWrite(LED_DEEP, HIGH);
    } else {
        digitalWrite(LED_DEEP, LOW);
    }

    if (isInShallowRange(sensorDepth)) {
        digitalWrite(LED_SHALLOW, HIGH);
    } else {
        digitalWrite(LED_SHALLOW, LOW);
    }
}

// ============== DATA RECORDING ==============
void recordDataPoint() {
    if (dataIndex >= MAX_DATA_POINTS) {
        Serial.println("WARNING: Data buffer full!");
        return;
    }

    float elapsedTime = (millis() - profileStartTime) / 1000.0;
    float sensorDepth = readDepthSensor();

    strncpy(myData[dataIndex].companyNum, COMPANY_NUMBER, sizeof(myData[dataIndex].companyNum) - 1);
    myData[dataIndex].companyNum[sizeof(myData[dataIndex].companyNum) - 1] = '\0';
    myData[dataIndex].time = elapsedTime;
    myData[dataIndex].pressure_kpa = readPressureSensor();
    myData[dataIndex].depth_m = sensorDepth;

    // Enhanced debug output showing all depth calculations
    Serial.print("DATA["); Serial.print(dataIndex); Serial.print("]: ");
    Serial.print(myData[dataIndex].companyNum); Serial.print(" | ");
    Serial.print(myData[dataIndex].time, 1); Serial.print("s | ");
    Serial.print(myData[dataIndex].pressure_kpa, 2); Serial.print("kPa | ");
    Serial.print("Sensor:"); Serial.print(sensorDepth, 3); Serial.print("m | ");
    Serial.print("Bot:"); Serial.print(getBottomDepth(sensorDepth), 3); Serial.print("m | ");
    Serial.print("Top:"); Serial.print(getTopDepth(sensorDepth), 3); Serial.println("m");

    dataIndex++;
}

// ============== MAIN LOOP ==============
void loop() {
    static unsigned long lastLoopTime = 0;
    unsigned long currentTime = millis();

    float sensorDepth = readDepthSensor();
    float bottomDepth = getBottomDepth(sensorDepth);
    float topDepth = getTopDepth(sensorDepth);

    // Update LED indicators
    updateLEDs(sensorDepth);

    // Record data every 5 seconds during active profiles
    if (currentState >= STATE_DESCENDING && currentState <= STATE_HOLD_SHALLOW) {
        if (currentTime - lastDataTime >= DATA_INTERVAL_MS) {
            lastDataTime = currentTime;
            recordDataPoint();
        }
    }

    // State machine
    switch (currentState) {
        case STATE_IDLE:
            break;

        case STATE_PRE_DIVE_TX:
            Serial.println("\n--- Sending pre-dive transmission ---");
            sendPreDivePacket();
            delay(500);

            teardownWifi();

            currentState = STATE_DESCENDING;
            stateStartTime = currentTime;
            lastDataTime = currentTime;
            recordDataPoint();
            moveFloatDown();
            Serial.print("\n=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" START ===");
            Serial.println("State: DESCENDING to 2.5m (bottom of float)");
            break;

        case STATE_DESCENDING:
            if (isInDeepRange(sensorDepth)) {
                stopFloat();
                currentState = STATE_HOLD_DEEP;
                holdStartTime = currentTime;
                holdInRange = true;
                holdInRangeStart = currentTime;
                Serial.println("\nState: HOLD_DEEP (30s at 2.5m)");
                Serial.print("Sensor depth: "); Serial.print(sensorDepth, 3); Serial.println("m");
                Serial.print("Bottom depth: "); Serial.print(bottomDepth, 3); Serial.println("m");
            }
            else if (motorsMoving && buoyEnc.getCount() <= TARGET_DIVE_PULSES) {
                stopFloat();
                Serial.println("WARNING: Max descent reached by encoder");
            }
            break;

        case STATE_HOLD_DEEP:
            if (isInDeepRange(sensorDepth)) {
                if (!holdInRange) {
                    holdInRange = true;
                    holdInRangeStart = currentTime;
                    Serial.println("Entered deep range, hold timer reset");
                    Serial.print("Bottom at: "); Serial.print(bottomDepth, 3);
                    Serial.print("m (valid: "); Serial.print(DEEP_MIN, 2);
                    Serial.print("-"); Serial.print(DEEP_MAX, 2); Serial.println("m)");
                }

                if (currentTime - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 2.5m COMPLETE <<<");
                    currentState = STATE_ASCENDING;
                    stateStartTime = currentTime;
                    holdInRange = false;
                    moveFloatUp();
                    Serial.println("State: ASCENDING to 40cm (top of float)");
                }
            } else {
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("WARNING: Drifted out of deep range!");
                    Serial.print(" Bottom: "); Serial.print(bottomDepth, 3); Serial.println("m");
                }

                if (bottomDepth < DEEP_MIN && !motorsMoving) {
                    moveFloatDown();
                } else if (bottomDepth > DEEP_MAX && !motorsMoving) {
                    moveFloatUp();
                }
            }
            break;

        case STATE_ASCENDING:
            if (isInShallowRange(sensorDepth)) {
                stopFloat();
                currentState = STATE_HOLD_SHALLOW;
                holdStartTime = currentTime;
                holdInRange = true;
                holdInRangeStart = currentTime;
                Serial.println("\nState: HOLD_SHALLOW (30s at 40cm)");
                Serial.print("Sensor depth: "); Serial.print(sensorDepth, 3); Serial.println("m");
                Serial.print("Top depth: "); Serial.print(topDepth, 3); Serial.println("m");
            }
            else if (topDepth < SHALLOW_MIN) {
                stopFloat();
                Serial.println("WARNING: Approaching surface! Stopping to prevent breach.");
                Serial.print("Top at: "); Serial.print(topDepth, 3); Serial.println("m");
                currentState = STATE_HOLD_SHALLOW;
                holdStartTime = currentTime;
                holdInRange = false;
            }
            else if (motorsMoving && buoyEnc.getCount() >= TARGET_SURFACE_PULSES) {
                stopFloat();
                Serial.println("WARNING: Max ascent reached by encoder");
            }
            break;

        case STATE_HOLD_SHALLOW:
            if (isInShallowRange(sensorDepth)) {
                if (!holdInRange) {
                    holdInRange = true;
                    holdInRangeStart = currentTime;
                    Serial.println("Entered shallow range, hold timer reset");
                    Serial.print("Top at: "); Serial.print(topDepth, 3);
                    Serial.print("m (valid: "); Serial.print(SHALLOW_MIN, 2);
                    Serial.print("-"); Serial.print(SHALLOW_MAX, 2); Serial.println("m)");
                }

                if (currentTime - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 40cm COMPLETE <<<");
                    Serial.print("=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" COMPLETE ===\n");

                    currentProfile++;

                    if (currentProfile < NUM_PROFILES) {
                        currentState = STATE_DESCENDING;
                        stateStartTime = currentTime;
                        buoyEnc.clearCount();
                        moveFloatDown();
                        Serial.print("\n=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" START ===");
                        Serial.println("State: DESCENDING to 2.5m (bottom of float)");
                    } else {
                        currentState = STATE_COMPLETE;
                        Serial.println("\n========================================");
                        Serial.println("ALL PROFILES COMPLETE");
                        Serial.print("Total data points recorded: "); Serial.println(dataIndex);
                        Serial.println("Ready for recovery.");
                        Serial.println("Send command 3.0 to transmit data.");
                        Serial.println("========================================");
                    }
                }
            } else {
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("WARNING: Drifted out of shallow range!");
                    Serial.print(" Top: "); Serial.print(topDepth, 3); Serial.println("m");
                }

                if (topDepth > SHALLOW_MAX && !motorsMoving) {
                    moveFloatUp();
                } else if (topDepth < SHALLOW_MIN && !motorsMoving) {
                    moveFloatDown();
                    delay(200);
                    stopFloat();
                }
            }
            break;

        case STATE_COMPLETE:
            if ((currentTime / 500) % 2 == 0) {
                digitalWrite(LED_DEEP, HIGH);
                digitalWrite(LED_SHALLOW, HIGH);
            } else {
                digitalWrite(LED_DEEP, LOW);
                digitalWrite(LED_SHALLOW, LOW);
            }
            break;

        case STATE_TRANSMITTING:
            Serial.println("\n--- Transmitting all profile data ---");
            sendAllData();
            currentState = STATE_IDLE;
            Serial.println("\nTransmission complete. Returning to IDLE.");
            break;
    }

    // Enhanced debug output every second during active states
    if (currentTime - lastLoopTime >= 1000 && currentState != STATE_IDLE && currentState != STATE_COMPLETE) {
        lastLoopTime = currentTime;

        if (currentState >= STATE_DESCENDING && currentState <= STATE_HOLD_SHALLOW) {
            float rawDepth = readDepthSensorRaw();

            Serial.print("[P"); Serial.print(currentProfile + 1); Serial.print("] ");
            Serial.print("Raw:"); Serial.print(rawDepth, 2); Serial.print("m ");
            Serial.print("Cor:"); Serial.print(sensorDepth, 2); Serial.print("m ");
            Serial.print("Bot:"); Serial.print(bottomDepth, 2); Serial.print("m ");
            Serial.print("Top:"); Serial.print(topDepth, 2); Serial.print("m ");

            if (holdInRange) {
                unsigned long holdElapsed = (currentTime - holdInRangeStart) / 1000;
                Serial.print("HOLD:"); Serial.print(holdElapsed); Serial.print("/30s ");
            }

            // Show which range we're in (if any)
            if (isInDeepRange(sensorDepth)) {
                Serial.print("[DEEP OK]");
            } else if (isInShallowRange(sensorDepth)) {
                Serial.print("[SHALLOW OK]");
            }
            Serial.println();
        }
    }
}
