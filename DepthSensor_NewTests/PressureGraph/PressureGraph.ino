#include <esp_now.h>
#include <WiFi.h>

#define MAX_DATA_POINTS 100
#define DATA_PER_PACKET 10
#define BUTTON_START 21
#define BUTTON_TRANSMIT 22  // Request data from float after recovery
#define BUTTON_MANUAL 23    // Toggle manual potentiometer motor control
#define PEN_IN 4

// ============== SENSOR OFFSETS (must match DepthSensor.ino) ==============
// Used to compute bottom/top depths from sensor reading for judge display
#define SENSOR_TO_BOTTOM_OFFSET 0.25    // 25.0 cm
#define SENSOR_TO_TOP_OFFSET 0.424      // 42.4 cm

// Scoring ranges (for reference)
#define DEEP_MIN 2.17
#define DEEP_MAX 2.83
#define SHALLOW_MIN 0.07
#define SHALLOW_MAX 0.73

//Surface MAC Address: 8C:4F:00:10:07:34
//Water MAC ADDRESS: F8:B3:B7:3E:FE:88
uint8_t broadcastAddress[] = {0xF8, 0xB3, 0xB7, 0x3E, 0xFE, 0x88};

//needed for broadcast purposes
esp_now_peer_info_t peerInfo;

//Structure for data transmission - must match DepthSensor.ino
typedef struct struct_message {
    char companyNum[8];   // Company identifier
    float time;           // Time in seconds
    float pressure_kpa;   // Pressure in kilopascals
    float depth_m;        // Depth in meters (sensor reading)
} struct_message;

// Storage for received profile data
struct_message receivedData[MAX_DATA_POINTS];
int receivedCount = 0;

bool startPressed = false;
bool wifiOn = false;
bool takeDownWifi = false;
bool connTakeDownOverride = false;
bool dataReceived = false;
bool manualModeActive = false;

void setup(){
    //Initialize Serial Monitor
    Serial.begin(9600);

    //BUTTON_START, pin setup
    pinMode(BUTTON_START, INPUT_PULLUP);
    pinMode(BUTTON_TRANSMIT, INPUT_PULLUP);
    pinMode(BUTTON_MANUAL, INPUT_PULLUP);

    //Delay to allow inital report to print and get output to a new line
    delay(250);
    Serial.println("");
    Serial.println("MATE Float 2026 - Mission Station");
    Serial.println("==================================");

    // Print offset info for judge communication
    Serial.println("\n--- SENSOR OFFSET INFO ---");
    Serial.print("Sensor to bottom: "); Serial.print(SENSOR_TO_BOTTOM_OFFSET * 100, 1); Serial.println(" cm");
    Serial.print("Sensor to top:    "); Serial.print(SENSOR_TO_TOP_OFFSET * 100, 1); Serial.println(" cm");
    Serial.print("Float height:     "); Serial.print((SENSOR_TO_BOTTOM_OFFSET + SENSOR_TO_TOP_OFFSET) * 100, 1); Serial.println(" cm");
    Serial.println("\n2.5m hold (bottom of float):");
    Serial.print(" Acceptable range:  "); Serial.print(DEEP_MIN, 2); Serial.print("m - "); Serial.print(DEEP_MAX, 2); Serial.println("m");
    Serial.print("  Offset range: "); Serial.print(DEEP_MIN - SENSOR_TO_BOTTOM_OFFSET, 2);
    Serial.print("m - "); Serial.print(DEEP_MAX - SENSOR_TO_BOTTOM_OFFSET, 2); Serial.println("m");
    Serial.println("\n40cm hold (top of float):");
    Serial.print("  Acceptable range:  "); Serial.print(SHALLOW_MIN, 2); Serial.print("m - "); Serial.print(SHALLOW_MAX, 2); Serial.println("m");
    Serial.print("  Offset range: "); Serial.print(SHALLOW_MIN + SENSOR_TO_TOP_OFFSET, 2);
    Serial.print("m - "); Serial.print(SHALLOW_MAX + SENSOR_TO_TOP_OFFSET, 2); Serial.println("m");
    Serial.println("--------------------------------------");

    Serial.println("\nD21 = START dive sequence");
    Serial.println("D22 = TRANSMIT data (request/reprint)");
    Serial.println("D23 = MANUAL mode toggle (potentiometer motor control)");
    Serial.println("Manual mode is OFF by default\n");
}

bool setupPeer(){
    // If already connected and peer exists, reuse the connection
    if(wifiOn){
        esp_now_peer_info_t existingPeer;
        if(esp_now_get_peer(broadcastAddress, &existingPeer) == ESP_OK){
            return false; // Already set up — nothing to do
        }
        // WiFi on but peer missing — tear down and rebuild cleanly
        DEFCON1();
    }

    //Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    wifiOn = true;

    //Init ESP-NOW
    if(esp_now_init() != ESP_OK){
        Serial.println("Error initializing ESP-NOW");
        return true;
    }
    else{
        Serial.println("ESP-NOW initializied successfully");
    }

    // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    //Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
    esp_now_register_recv_cb(OnDataRecv);

    // Register peer
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return true;
    }
    else{
        Serial.println("Peer added successfully");
    }

    return false;
}

void DEFCON1(){
    //Remove Peer
    esp_now_peer_num_t peer_count;
    esp_err_t del_result;
    esp_now_get_peer_num(&peer_count);
    if(peer_count.total_num > 0){
        del_result = esp_now_del_peer(broadcastAddress);
        if(del_result == ESP_OK){
            Serial.println("Peer removed successfully via direct MAC address");
        }
        else{
            Serial.println("Failed to remove peer directly: " + String(del_result));
        }
    }
    else{
        Serial.println("No peers to remove.");
    }

    //Clear esp_now to re-init it and have a clean client add
    if(esp_now_deinit() == ESP_OK){
        Serial.println("ESP-NOW deinitialized successfully");
    }
    else{
        Serial.println("ESP-NOW was not active or deinit failed");
    }

    //Disconnect WIFI
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    //Var change
    wifiOn = false;
}

//Send signal for dive sequence (command 1.0)
void sendStartSignal(){
    if(setupPeer()){
        return;
    }

    struct_message reqStartPacket;
    memset(&reqStartPacket, 0, sizeof(reqStartPacket));
    reqStartPacket.time = 1.0;
    reqStartPacket.pressure_kpa = 0.0;
    reqStartPacket.depth_m = 0.0;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &reqStartPacket, sizeof(reqStartPacket));

    if(result == ESP_OK){
        Serial.println("Sent start signal with success");
        startPressed = true;

        // Clear any previously received data
        receivedCount = 0;
        dataReceived = false;
        memset(receivedData, 0, sizeof(receivedData));
    }
    else{
        Serial.println("Error sending start signal");
    }
}

//Send signal to request data transmission after recovery (command 3.0)
void sendTransmitSignal(){
    if(setupPeer()){
        return;
    }

    struct_message reqTransmitPacket;
    memset(&reqTransmitPacket, 0, sizeof(reqTransmitPacket));
    reqTransmitPacket.time = 3.0;
    reqTransmitPacket.pressure_kpa = 0.0;
    reqTransmitPacket.depth_m = 0.0;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &reqTransmitPacket, sizeof(reqTransmitPacket));

    if(result == ESP_OK){
        Serial.println("Sent transmit request with success");
        Serial.println("Waiting for data from float...");
    }
    else{
        Serial.println("Error sending transmit request");
    }
}

//Send signal to move motor due to potentiometer data (command 2.0)
void sendPotentiometerReading(int potentiometerVal){
    if(setupPeer()){
        return;
    }

    struct_message reqMotorPacket;
    memset(&reqMotorPacket, 0, sizeof(reqMotorPacket));
    reqMotorPacket.time = 2.0;
    reqMotorPacket.pressure_kpa = 0.0;
    reqMotorPacket.depth_m = (float)potentiometerVal;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &reqMotorPacket, sizeof(reqMotorPacket));

    if (result == ESP_OK){
        Serial.println("Sent potentiometer reading with success");
    }
    else{
        Serial.println("Error sending potentiometer reading: " + String(esp_err_to_name(result)));
    }
}

//Callback when data is sent. Disable peer so we can analog read again
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status){
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Packet Delivery Success" : "Packet Delivery Failure");

    if(connTakeDownOverride){
        connTakeDownOverride = false;
    }
    else{
        takeDownWifi = true;
    }
}

//Receive data from float
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len){
    int singleSize = sizeof(struct_message);
    int packetSize = sizeof(struct_message) * DATA_PER_PACKET;

    // Single packet (pre-dive transmission)
    if(len == singleSize){
        struct_message singlePacket;
        memcpy(&singlePacket, incomingData, singleSize);

        float bottomDepth = singlePacket.depth_m + SENSOR_TO_BOTTOM_OFFSET;
        float topDepth = singlePacket.depth_m - SENSOR_TO_TOP_OFFSET;

        Serial.println("\n--- Pre-Dive Packet Received ---");
        Serial.print("Company: "); Serial.println(singlePacket.companyNum);
        Serial.print("Time: "); Serial.print(singlePacket.time, 2); Serial.println("s");
        Serial.print("Pressure: "); Serial.print(singlePacket.pressure_kpa, 2); Serial.println(" kPa");
        Serial.print("Sensor depth: "); Serial.print(singlePacket.depth_m, 4); Serial.println(" m");
        Serial.print("Bottom depth: "); Serial.print(bottomDepth, 4); Serial.println(" m");
        Serial.print("Top depth: "); Serial.print(topDepth, 4); Serial.println(" m");
        Serial.println("--------------------------------");
    }
    // Data packet array (profile data from float)
    else if(len == packetSize){
        struct_message packetData[DATA_PER_PACKET];
        memcpy(&packetData, incomingData, packetSize);

        // Print CSV header on first data packet received
        if(!dataReceived){
            dataReceived = true;
            Serial.println("\n--- Profile Data Received ---");
            Serial.println("Company,Time_s,Pressure_kPa,Sensor_m,Bottom_m,Top_m");
        }

        // Store and print each data point
        for(int i = 0; i < DATA_PER_PACKET; i++){
            // Skip empty/zeroed entries (padding in last packet)
            if(packetData[i].time == 0.0 && packetData[i].depth_m == 0.0 && packetData[i].pressure_kpa == 0.0){
                continue;
            }

            // Store if room remains
            if(receivedCount < MAX_DATA_POINTS){
                receivedData[receivedCount] = packetData[i];
                receivedCount++;
            }

            // Compute physical depths from sensor reading
            float bottomDepth = packetData[i].depth_m + SENSOR_TO_BOTTOM_OFFSET;
            float topDepth = packetData[i].depth_m - SENSOR_TO_TOP_OFFSET;

            // Print as CSV with computed columns
            Serial.print(packetData[i].companyNum);
            Serial.print(",");
            Serial.print(packetData[i].time, 2);
            Serial.print(",");
            Serial.print(packetData[i].pressure_kpa, 2);
            Serial.print(",");
            Serial.print(packetData[i].depth_m, 4);
            Serial.print(",");
            Serial.print(bottomDepth, 4);
            Serial.print(",");
            Serial.println(topDepth, 4);
        }

        Serial.print("(Total received so far: ");
        Serial.print(receivedCount);
        Serial.println(" data points)");
    }
    else{
        Serial.print("WARNING: Received unknown packet size: ");
        Serial.println(len);
    }
}

// Print all stored data with computed depths for judge review
void printAllData(){
    if(receivedCount == 0){
        Serial.println("No data to display.");
        return;
    }

    Serial.println("\n========================================");
    Serial.println("COMPLETE PROFILE DATA");
    Serial.print("Total data points: ");
    Serial.println(receivedCount);
    Serial.println("========================================");
    Serial.println("Company,Time_s,Pressure_kPa,Sensor_m,Bottom_m,Top_m");

    for(int i = 0; i < receivedCount; i++){
        float bottomDepth = receivedData[i].depth_m + SENSOR_TO_BOTTOM_OFFSET;
        float topDepth = receivedData[i].depth_m - SENSOR_TO_TOP_OFFSET;

        Serial.print(receivedData[i].companyNum);
        Serial.print(",");
        Serial.print(receivedData[i].time, 2);
        Serial.print(",");
        Serial.print(receivedData[i].pressure_kpa, 2);
        Serial.print(",");
        Serial.print(receivedData[i].depth_m, 4);
        Serial.print(",");
        Serial.print(bottomDepth, 4);
        Serial.print(",");
        Serial.println(topDepth, 4);
    }

    Serial.println("\n========================================");
    Serial.print("Data points for graphing: ");
    Serial.println(receivedCount);
    if(receivedCount >= 20){
        Serial.println("Meets minimum 20 data point requirement.");
    }
    else{
        Serial.println("WARNING: Below 20 data point minimum for graphing.");
    }

    // Check for 7 sequential deep-hold packets
    Serial.println("\n--- HOLD VERIFICATION ---");
    Serial.println("Deep hold (bottom 2.17-2.83m):");
    int deepSeq = 0;
    int deepMaxSeq = 0;
    for(int i = 0; i < receivedCount; i++){
        float bottom = receivedData[i].depth_m + SENSOR_TO_BOTTOM_OFFSET;
        if(bottom >= DEEP_MIN && bottom <= DEEP_MAX){
            deepSeq++;
            if(deepSeq > deepMaxSeq) deepMaxSeq = deepSeq;
        }
        else{
            deepSeq = 0;
        }
    }
    Serial.print("  Max sequential in-range packets: ");
    Serial.print(deepMaxSeq);
    Serial.println(deepMaxSeq >= 7 ? " (PASS - need 7)" : " (FAIL - need 7)");

    // Check for 7 sequential shallow-hold packets
    Serial.println("Shallow hold (top 0.07-0.73m):");
    int shallowSeq = 0;
    int shallowMaxSeq = 0;
    for(int i = 0; i < receivedCount; i++){
        float top = receivedData[i].depth_m - SENSOR_TO_TOP_OFFSET;
        if(top >= SHALLOW_MIN && top <= SHALLOW_MAX){
            shallowSeq++;
            if(shallowSeq > shallowMaxSeq) shallowMaxSeq = shallowSeq;
        }
        else{
            shallowSeq = 0;
        }
    }
    Serial.print("  Max sequential in-range packets: ");
    Serial.print(shallowMaxSeq);
    Serial.println(shallowMaxSeq >= 7 ? " (PASS - need 7)" : " (FAIL - need 7)");

    Serial.println("========================================");
}

//Button debouncing code
void debounceBool(bool& buttonVALUE, bool currentBUTTONvalue, bool& bufferBUTTONvalue, long& debounceStart, int debounceTime){
    if(bufferBUTTONvalue != currentBUTTONvalue){
        debounceStart = millis();
    }
    if(millis() - debounceStart > debounceTime){
        buttonVALUE = currentBUTTONvalue;
    }
    bufferBUTTONvalue = currentBUTTONvalue;
}

//Poten debouncing code
void debounceInt(int& buttonVALUE, int currentBUTTONvalue, int& bufferBUTTONvalue, long& debounceStart, int debounceTime){
    if(bufferBUTTONvalue != currentBUTTONvalue){
        debounceStart = millis();
    }
    if(millis() - debounceStart > debounceTime){
        buttonVALUE = currentBUTTONvalue;
    }
    bufferBUTTONvalue = currentBUTTONvalue;
}

//Gets a reading from the potentiometer
int readPotentiometer(){
    int analogReading = analogRead(PEN_IN);
    float voltage = float(analogReading) / 4095 * 3.3;

    if(voltage < 1.1){
        return -1;
    }
    else if(voltage > 2.2){
        return 1;
    }
    else{
        return 0;
    }
}

//Button(s) decision and action code
void loop(){
    static bool BUTTON_STARTval = HIGH;
    static bool currentBUTTON_STARTvalue = HIGH;
    static bool BUTTON_STARTbuffer = HIGH;
    static long debounceSTARTstart = 0;

    static bool BUTTON_TRANSMITval = HIGH;
    static bool currentBUTTON_TRANSMITvalue = HIGH;
    static bool BUTTON_TRANSMITbuffer = HIGH;
    static long debounceTRANSMITstart = 0;

    static bool BUTTON_MANUALval = HIGH;
    static bool currentBUTTON_MANUALvalue = HIGH;
    static bool BUTTON_MANUALbuffer = HIGH;
    static long debounceMANUALstart = 0;

    static int debounceTime = 50;

    static bool canRequestStart = true;
    static bool canRequestTransmit = true;
    static bool canToggleManual = true;

    static int potentiometerVal = 0;
    static int potentiometerReading = 0;
    static int potentiometerBuffer = 0;
    static long debouncePotenStart = 0;
    static int potentPrevSend = 0;

    if(takeDownWifi){
        DEFCON1();
        takeDownWifi = false;

        // After transmit signal is sent and WiFi is torn down,
        // re-setup WiFi to listen for incoming data
        if(dataReceived || startPressed){
            delay(100);
            setupPeer();
            connTakeDownOverride = true;
        }
    }

    //read the current values of all buttons
    currentBUTTON_STARTvalue = digitalRead(BUTTON_START);
    currentBUTTON_TRANSMITvalue = digitalRead(BUTTON_TRANSMIT);
    currentBUTTON_MANUALvalue = digitalRead(BUTTON_MANUAL);

    //debounce all button inputs
    debounceBool(BUTTON_STARTval, currentBUTTON_STARTvalue, BUTTON_STARTbuffer, debounceSTARTstart, debounceTime);
    debounceBool(BUTTON_TRANSMITval, currentBUTTON_TRANSMITvalue, BUTTON_TRANSMITbuffer, debounceTRANSMITstart, debounceTime);
    debounceBool(BUTTON_MANUALval, currentBUTTON_MANUALvalue, BUTTON_MANUALbuffer, debounceMANUALstart, debounceTime);

    // ---- MANUAL MODE TOGGLE (D23) ----
    if(BUTTON_MANUALval == LOW && canToggleManual){
        canToggleManual = false;
        manualModeActive = !manualModeActive;

        if(manualModeActive){
            Serial.println("\n>>> MANUAL MODE ON <<<");
            Serial.println("Potentiometer now controls motor.");
            // Initialize potentiometer tracking
            potentPrevSend = 0;
        }
        else{
            Serial.println("\n>>> MANUAL MODE OFF <<<");
            // Send stop command immediately when disabling manual mode
            sendPotentiometerReading(0);
            Serial.println("Motor stop sent.");
        }
    }
    if(BUTTON_MANUALval == HIGH){
        canToggleManual = true;
    }

    // ---- POTENTIOMETER (only when manual mode is active) ----
    if(manualModeActive){
        if(!wifiOn){
            potentiometerReading = readPotentiometer();
        }

        debounceInt(potentiometerVal, potentiometerReading, potentiometerBuffer, debouncePotenStart, debounceTime);

        if(potentPrevSend != potentiometerVal){
            sendPotentiometerReading(potentiometerVal);
            Serial.println("Potentiometer Value: " + String(potentiometerVal));
            potentPrevSend = potentiometerVal;
        }
    }

    // ---- START DIVE (D21) ----
    if(BUTTON_STARTval == LOW && canRequestStart){
        canRequestStart = false;

        if(manualModeActive){
            Serial.println("WARNING: Disable manual mode before starting dive.");
        }
        else{
            // Send motor stop before starting dive
            sendPotentiometerReading(0);
            Serial.println("PotentiometerValue: 0");

            //wait for potentiometer to be fully sent before resetting and sending start
            while(!takeDownWifi){
                //wait
            }
            DEFCON1();
            takeDownWifi = false;

            connTakeDownOverride = true;
            sendStartSignal();
        }
    }
    if(BUTTON_STARTval == HIGH){
        canRequestStart = true;
    }

    // ---- TRANSMIT DATA (D22) ----
    if(BUTTON_TRANSMITval == LOW && canRequestTransmit){
        canRequestTransmit = false;

        if(dataReceived && receivedCount > 0){
            // Data already received - reprint it
            printAllData();
        }
        else{
            // No data yet - send transmit request to float
            connTakeDownOverride = true;
            sendTransmitSignal();
            Serial.println("\nTransmit request sent. Listening for data...");
        }
    }
    if(BUTTON_TRANSMITval == HIGH){
        canRequestTransmit = true;
    }
}
