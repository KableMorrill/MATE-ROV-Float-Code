#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

// ============== PIN DEFINITIONS ==============
#define DEPTH_PIN 25
#define IN1 18  // buoyancy motor pins (H-bridge)
#define IN2 19
#define ENC1 17
#define ENC2 4
#define LED_DEEP 2      // Blue LED - indicates at 2.5m range
#define LED_SHALLOW 15  // Green LED - indicates at 40cm range

// ============== PWM CONFIGURATION ==============
// ESP32 LEDC channels for variable motor speed
// This allows us to control motor speed instead of just on/off
// which we'll need for smooth approach to targets and gentle corrections.
#define PWM_CHANNEL_IN1 0
#define PWM_CHANNEL_IN2 1
#define PWM_FREQ 1000      // 1kHz PWM frequency
#define PWM_RESOLUTION 8   // 8-bit = 0-255

// ============== MOTOR SPEED PARAMETERS ==============
#define MOTOR_MAX_SPEED 255   // Full speed PWM duty
#define MOTOR_MIN_SPEED 60    // Minimum PWM that still turns the motor
// The approach zone defines the distance (meters) from target at which
// the motor begins to slow down. Outside this zone = full speed.
#define APPROACH_ZONE 0.50
// Stop the motor this many meters before the target center to let
// buoyancy drift carry the float the rest of the way.
// Tune this based on testing - larger value = more coast distance.
#define STOP_EARLY_DISTANCE 0.08
// During hold corrections, use gentler speeds
#define HOLD_CORRECTION_MAX_SPEED 120
#define HOLD_CORRECTION_MIN_SPEED 50

// ============== MOTOR/ENCODER SETTINGS ==============
#define PULSES_PER_REV 2600
#define TARGET_DIVE_PULSES (PULSES_PER_REV * -8)
#define TARGET_SURFACE_PULSES 0

// ============== COMPETITION PARAMETERS ==============
// Depth targets (in meters)
#define TARGET_DEEP 2.5       // Deep hold depth (bottom of float)
#define TARGET_SHALLOW 0.40   // Shallow hold depth (top of float)
#define DEPTH_TOLERANCE 0.33  // +/- 33cm tolerance

// Calculated ranges (what the JUDGE measures)
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
#define EGADS_SPECIFIC_GRAVITY 1.025
#define EGADS_CORRECTION_FACTOR (1.0 / EGADS_SPECIFIC_GRAVITY)

// ============== SENSOR OFFSET ==============
// The depth sensor is mounted on the side of the float.
//
//     +---------+  <-- TOP of float
//     |         |
//     |  42.4cm |
//     |         |
//     |    *    |  <-- Depth sensor
//     |         |
//     |  25.0cm |
//     |         |
//     +---------+  <-- BOTTOM of float
//
// Total float height: 25.0 + 42.4 = 67.4 cm
//
#define SENSOR_TO_BOTTOM_OFFSET 0.25    // 25.0 cm below sensor to bottom
#define SENSOR_TO_TOP_OFFSET 0.424      // 42.4 cm above sensor to top

// Sensor readings that correspond to target positions:
// Deep hold:    bottom at 2.5m  -> sensor reads 2.5 - 0.25 = 2.25m
// Shallow hold: top at 0.40m   -> sensor reads 0.40 + 0.424 = 0.824m
//
// Sensor valid ranges for scoring:
// Deep hold:    bottom 2.17-2.83m -> sensor 1.92-2.58m
// Shallow hold: top 0.07-0.73m   -> sensor 0.494-1.154m

// ============== COMPANY IDENTIFIER ==============
#define COMPANY_NUMBER "PN00"  // TODO: Update with MATE-assigned company number

// ============== ESP-NOW CONFIGURATION ==============
uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x10, 0x07, 0x34};

esp_now_peer_info_t peerInfo;

// ============== DATA STRUCTURES ==============
typedef struct struct_message {
    char companyNum[8];
    float time;
    float pressure_kpa;
    float depth_m;
} struct_message;

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
int currentMotorSpeed = 0;

ESP32Encoder buoyEnc;

// Precomputed sensor targets (avoids recalculating every loop)
float sensorTargetDeep;
float sensorTargetShallow;
float sensorDeepMin;
float sensorDeepMax;
float sensorShallowMin;
float sensorShallowMax;

// ============== SETUP ==============
void setup() {
    Serial.begin(9600);
    Serial.println("MATE Float 2026 - Ice Tank Version");
    Serial.println("==================================");

    // PWM setup for variable motor speed
    ledcSetup(PWM_CHANNEL_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(IN1, PWM_CHANNEL_IN1);
    ledcAttachPin(IN2, PWM_CHANNEL_IN2);
    ledcWrite(PWM_CHANNEL_IN1, 0);
    ledcWrite(PWM_CHANNEL_IN2, 0);

    // LED indicators
    pinMode(LED_DEEP, OUTPUT);
    pinMode(LED_SHALLOW, OUTPUT);
    digitalWrite(LED_DEEP, LOW);
    digitalWrite(LED_SHALLOW, LOW);

    // Encoder setup
    buoyEnc.attachHalfQuad(ENC1, ENC2);
    buoyEnc.clearCount();

    // Precompute sensor target depths
    sensorTargetDeep = TARGET_DEEP - SENSOR_TO_BOTTOM_OFFSET;       // 2.25m
    sensorTargetShallow = TARGET_SHALLOW + SENSOR_TO_TOP_OFFSET;    // 0.824m
    sensorDeepMin = DEEP_MIN - SENSOR_TO_BOTTOM_OFFSET;             // 1.92m
    sensorDeepMax = DEEP_MAX - SENSOR_TO_BOTTOM_OFFSET;             // 2.58m
    sensorShallowMin = SHALLOW_MIN + SENSOR_TO_TOP_OFFSET;          // 0.494m
    sensorShallowMax = SHALLOW_MAX + SENSOR_TO_TOP_OFFSET;          // 1.154m

    // Initialize WiFi/ESP-NOW
    if (setupPeer()) {
        Serial.println("ERROR: Failed to initialize ESP-NOW");
        return;
    }

    // Print configuration
    Serial.println("\nConfiguration:");
    Serial.print("  Company: "); Serial.println(COMPANY_NUMBER);
    Serial.print("  Float height: "); Serial.print((SENSOR_TO_BOTTOM_OFFSET + SENSOR_TO_TOP_OFFSET) * 100, 1); Serial.println(" cm");
    Serial.print("  Sensor to bottom: "); Serial.print(SENSOR_TO_BOTTOM_OFFSET * 100, 1); Serial.println(" cm");
    Serial.print("  Sensor to top: "); Serial.print(SENSOR_TO_TOP_OFFSET * 100, 1); Serial.println(" cm");
    Serial.println("\nScoring ranges (sensor reading):");
    Serial.print("  Deep hold:    "); Serial.print(sensorDeepMin, 2); Serial.print("m - "); Serial.print(sensorDeepMax, 2); Serial.println("m");
    Serial.print("  Shallow hold: "); Serial.print(sensorShallowMin, 2); Serial.print("m - "); Serial.print(sensorShallowMax, 2); Serial.println("m");
    Serial.print("  Approach zone: "); Serial.print(APPROACH_ZONE * 100, 0); Serial.println(" cm");
    Serial.print("  Stop-early distance: "); Serial.print(STOP_EARLY_DISTANCE * 100, 0); Serial.println(" cm");
    Serial.println("\nReady. Waiting for start command...");
}

// ============== MOTOR SPEED CALCULATION ==============
// Returns PWM duty 0-255 based on distance to target.
// Uses linear ramp: full speed far away, minimum speed near target,
// zero at stop-early distance to allow buoyancy drift.
int calculateApproachSpeed(float distanceToTarget) {
    if (distanceToTarget <= STOP_EARLY_DISTANCE) {
        return 0;
    }
    if (distanceToTarget >= APPROACH_ZONE) {
        return MOTOR_MAX_SPEED;
    }

    // Linear interpolation between MIN and MAX over the approach zone
    float ratio = (distanceToTarget - STOP_EARLY_DISTANCE) / (APPROACH_ZONE - STOP_EARLY_DISTANCE);
    return MOTOR_MIN_SPEED + (int)(ratio * (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED));
}

// Gentler speed calculation for hold-state corrections
int calculateCorrectionSpeed(float distanceFromRange) {
    if (distanceFromRange <= 0.01) {
        return 0;
    }
    if (distanceFromRange >= APPROACH_ZONE) {
        return HOLD_CORRECTION_MAX_SPEED;
    }

    float ratio = distanceFromRange / APPROACH_ZONE;
    return HOLD_CORRECTION_MIN_SPEED + (int)(ratio * (HOLD_CORRECTION_MAX_SPEED - HOLD_CORRECTION_MIN_SPEED));
}

// ============== MOTOR CONTROL (PWM) ==============
void moveFloatDown(int speed) {
    ledcWrite(PWM_CHANNEL_IN1, 0);
    ledcWrite(PWM_CHANNEL_IN2, speed);
    motorsMoving = (speed > 0);
    descending = true;
    currentMotorSpeed = speed;
}

void moveFloatUp(int speed) {
    ledcWrite(PWM_CHANNEL_IN1, speed);
    ledcWrite(PWM_CHANNEL_IN2, 0);
    motorsMoving = (speed > 0);
    descending = false;
    currentMotorSpeed = speed;
}

void stopFloat() {
    ledcWrite(PWM_CHANNEL_IN1, 0);
    ledcWrite(PWM_CHANNEL_IN2, 0);
    motorsMoving = false;
    currentMotorSpeed = 0;
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

    for (int i = 0; i < DATA_PER_PACKET; i++) {
        memset(&dataPacket[i], 0, sizeof(struct_message));
    }

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
    // Manual motor control (potentiometer) - uses full speed
    else if (packetData.time == 2.0) {
        if (packetData.depth_m == -1.0) {
            moveFloatUp(MOTOR_MAX_SPEED);
        } else if (packetData.depth_m == 0.0) {
            stopFloat();
        } else if (packetData.depth_m == 1.0) {
            moveFloatDown(MOTOR_MAX_SPEED);
        }
    }
    else if (packetData.time == 3.0) {
        Serial.println("\n>>> TRANSMIT COMMAND RECEIVED <<<");
        currentState = STATE_TRANSMITTING;
    }
}

// ============== SENSOR READINGS ==============
float readDepthSensor() {
    float analogReading = analogRead(DEPTH_PIN);
    float voltage = (analogReading / 4095.0) * 3.3;
    float mAmps = (voltage / 150.0) * 1000;
    float depth_raw = (312.5 * (mAmps - 4)) / 1000;

    float depth_corrected = depth_raw * EGADS_CORRECTION_FACTOR;

    return depth_corrected;
}

float readPressureSensor() {
    float depth = readDepthSensor();
    float pressure_kpa = 101.325 + (10.055 * depth);
    return pressure_kpa;
}

float getBottomDepth(float sensorDepth) {
    return sensorDepth + SENSOR_TO_BOTTOM_OFFSET;
}

float getTopDepth(float sensorDepth) {
    return sensorDepth - SENSOR_TO_TOP_OFFSET;
}

// ============== DEPTH RANGE CHECKING ==============
// These check using sensor-relative ranges (offset already applied)
bool isInDeepRange(float sensorDepth) {
    return (sensorDepth >= sensorDeepMin && sensorDepth <= sensorDeepMax);
}

bool isInShallowRange(float sensorDepth) {
    return (sensorDepth >= sensorShallowMin && sensorDepth <= sensorShallowMax);
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

    strncpy(myData[dataIndex].companyNum, COMPANY_NUMBER, sizeof(myData[dataIndex].companyNum) - 1);
    myData[dataIndex].companyNum[sizeof(myData[dataIndex].companyNum) - 1] = '\0';
    myData[dataIndex].time = elapsedTime;
    myData[dataIndex].pressure_kpa = readPressureSensor();
    myData[dataIndex].depth_m = readDepthSensor();

    Serial.print("DATA["); Serial.print(dataIndex); Serial.print("]: ");
    Serial.print(myData[dataIndex].companyNum); Serial.print(" ");
    Serial.print(myData[dataIndex].time, 1); Serial.print("s ");
    Serial.print(myData[dataIndex].pressure_kpa, 2); Serial.print("kPa ");
    Serial.print(myData[dataIndex].depth_m, 3); Serial.println("m");

    dataIndex++;
}

// ============== MAIN LOOP ==============
void loop() {
    static unsigned long lastLoopTime = 0;
    unsigned long currentTime = millis();

    float sensorDepth = readDepthSensor();
    float bottomDepth = getBottomDepth(sensorDepth);
    float topDepth = getTopDepth(sensorDepth);

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
            moveFloatDown(MOTOR_MAX_SPEED);
            Serial.print("\n=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" START ===");
            Serial.println("State: DESCENDING to 2.5m");
            break;

        case STATE_DESCENDING: {
            // Distance from current sensor reading to deep target
            float distToTarget = sensorTargetDeep - sensorDepth;

            if (distToTarget > 0) {
                // Still above target - calculate approach speed
                int speed = calculateApproachSpeed(distToTarget);
                if (speed > 0) {
                    moveFloatDown(speed);
                } else {
                    // Within stop-early zone: stop and let buoyancy drift
                    stopFloat();
                }
            }

            // Check if we've entered the valid range
            if (isInDeepRange(sensorDepth)) {
                stopFloat();
                currentState = STATE_HOLD_DEEP;
                holdStartTime = currentTime;
                holdInRange = true;
                holdInRangeStart = currentTime;
                Serial.println("\nState: HOLD_DEEP (30s at 2.5m)");
                Serial.print("Sensor: "); Serial.print(sensorDepth, 3);
                Serial.print("m  Bottom: "); Serial.print(bottomDepth, 3); Serial.println("m");
            }
            // Safety: encoder limit
            else if (motorsMoving && buoyEnc.getCount() <= TARGET_DIVE_PULSES) {
                stopFloat();
                Serial.println("WARNING: Max descent reached by encoder");
            }
            break;
        }

        case STATE_HOLD_DEEP: {
            if (isInDeepRange(sensorDepth)) {
                if (!holdInRange) {
                    holdInRange = true;
                    holdInRangeStart = currentTime;
                    stopFloat();
                    Serial.println("Entered deep range, hold timer reset");
                }

                if (currentTime - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 2.5m COMPLETE <<<");
                    currentState = STATE_ASCENDING;
                    stateStartTime = currentTime;
                    holdInRange = false;
                    buoyEnc.clearCount();
                    moveFloatUp(MOTOR_MAX_SPEED);
                    Serial.println("State: ASCENDING to 40cm");
                }
            } else {
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("WARNING: Drifted out! Sensor: ");
                    Serial.print(sensorDepth, 3); Serial.print("m  Valid: ");
                    Serial.print(sensorDeepMin, 2); Serial.print("-");
                    Serial.print(sensorDeepMax, 2); Serial.println("m");
                }

                // Gentle correction using proportional speed
                if (sensorDepth < sensorDeepMin) {
                    float dist = sensorDeepMin - sensorDepth;
                    int speed = calculateCorrectionSpeed(dist);
                    if (speed > 0) moveFloatDown(speed);
                } else if (sensorDepth > sensorDeepMax) {
                    float dist = sensorDepth - sensorDeepMax;
                    int speed = calculateCorrectionSpeed(dist);
                    if (speed > 0) moveFloatUp(speed);
                }
            }
            break;
        }

        case STATE_ASCENDING: {
            // Distance from current sensor reading to shallow target
            float distToTarget = sensorDepth - sensorTargetShallow;

            if (distToTarget > 0) {
                int speed = calculateApproachSpeed(distToTarget);
                if (speed > 0) {
                    moveFloatUp(speed);
                } else {
                    stopFloat();
                }
            }

            if (isInShallowRange(sensorDepth)) {
                stopFloat();
                currentState = STATE_HOLD_SHALLOW;
                holdStartTime = currentTime;
                holdInRange = true;
                holdInRangeStart = currentTime;
                Serial.println("\nState: HOLD_SHALLOW (30s at 40cm)");
                Serial.print("Sensor: "); Serial.print(sensorDepth, 3);
                Serial.print("m  Top: "); Serial.print(topDepth, 3); Serial.println("m");
            }
            // Surface breach prevention
            else if (sensorDepth < sensorShallowMin) {
                stopFloat();
                Serial.println("WARNING: Above shallow range! Stopping to prevent breach.");
                Serial.print("Top at: "); Serial.print(topDepth, 3); Serial.println("m");
                // Try to descend back into range
                float dist = sensorShallowMin - sensorDepth;
                int speed = calculateCorrectionSpeed(dist);
                if (speed > 0) moveFloatDown(speed);
            }
            // Safety: encoder limit
            else if (motorsMoving && buoyEnc.getCount() >= TARGET_SURFACE_PULSES) {
                stopFloat();
                Serial.println("WARNING: Max ascent reached by encoder");
            }
            break;
        }

        case STATE_HOLD_SHALLOW: {
            if (isInShallowRange(sensorDepth)) {
                if (!holdInRange) {
                    holdInRange = true;
                    holdInRangeStart = currentTime;
                    stopFloat();
                    Serial.println("Entered shallow range, hold timer reset");
                }

                if (currentTime - holdInRangeStart >= HOLD_DURATION_MS) {
                    Serial.println("\n>>> 30 SECONDS AT 40cm COMPLETE <<<");
                    Serial.print("=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" COMPLETE ===\n");

                    currentProfile++;

                    if (currentProfile < NUM_PROFILES) {
                        currentState = STATE_DESCENDING;
                        stateStartTime = currentTime;
                        buoyEnc.clearCount();
                        moveFloatDown(MOTOR_MAX_SPEED);
                        Serial.print("\n=== PROFILE "); Serial.print(currentProfile + 1); Serial.println(" START ===");
                        Serial.println("State: DESCENDING to 2.5m");
                    } else {
                        currentState = STATE_COMPLETE;
                        Serial.println("\n========================================");
                        Serial.println("ALL PROFILES COMPLETE");
                        Serial.print("Total data points recorded: "); Serial.println(dataIndex);
                        Serial.println("Ready for recovery. Send command 3.0 to transmit data.");
                        Serial.println("========================================");
                    }
                }
            } else {
                if (holdInRange) {
                    holdInRange = false;
                    Serial.print("WARNING: Drifted out! Sensor: ");
                    Serial.print(sensorDepth, 3); Serial.print("m  Valid: ");
                    Serial.print(sensorShallowMin, 2); Serial.print("-");
                    Serial.print(sensorShallowMax, 2); Serial.println("m");
                }

                // Gentle correction
                if (sensorDepth > sensorShallowMax) {
                    // Too deep - ascend gently
                    float dist = sensorDepth - sensorShallowMax;
                    int speed = calculateCorrectionSpeed(dist);
                    if (speed > 0) moveFloatUp(speed);
                } else if (sensorDepth < sensorShallowMin) {
                    // Too shallow (risk of surface breach!) - descend gently
                    float dist = sensorShallowMin - sensorDepth;
                    int speed = calculateCorrectionSpeed(dist);
                    if (speed > 0) moveFloatDown(speed);
                }
            }
            break;
        }

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

    // Debug output every second during active states
    if (currentTime - lastLoopTime >= 1000 && currentState != STATE_IDLE) {
        lastLoopTime = currentTime;
        if (currentState >= STATE_DESCENDING && currentState <= STATE_HOLD_SHALLOW) {
            Serial.print("[P"); Serial.print(currentProfile + 1); Serial.print("] ");
            Serial.print("S:"); Serial.print(sensorDepth, 2); Serial.print("m ");
            Serial.print("Bot:"); Serial.print(bottomDepth, 2); Serial.print("m ");
            Serial.print("Top:"); Serial.print(topDepth, 2); Serial.print("m ");
            Serial.print("Spd:"); Serial.print(currentMotorSpeed); Serial.print(" ");
            if (holdInRange) {
                unsigned long holdElapsed = (currentTime - holdInRangeStart) / 1000;
                Serial.print("HOLD:"); Serial.print(holdElapsed); Serial.print("/30s");
            }
            Serial.println();
        }
    }
}
