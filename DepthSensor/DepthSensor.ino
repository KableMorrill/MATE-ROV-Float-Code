#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

#define DEPTH_PIN 25
#define IN1 18 // buoyancy motor pins
#define IN2 19
#define ENC1 17
#define ENC2 4
#define PULSES_PER_REV 2600
#define TARGET_DIVE_PULSES PULSES_PER_REV * -8
#define TARGET_SURFACE_PULSES 0
#define DATA_PER_PACKET 30
#define SECONDS_UNDERWATER 120
#define ARRAYS_NEEDED ((SECONDS_UNDERWATER % DATA_PER_PACKET == 0) ? SECONDS_UNDERWATER / DATA_PER_PACKET : SECONDS_UNDERWATER / DATA_PER_PACKET + 1)
#define NUM_DIVES 2
#define DIVE_FLOOR .5 // was 3
#define DIVE_CEILING 0 // was .666
#define B_LED 2

//Surface MAC Address: 8C:4F:00:10:07:34
//Water MAC ADDRESS: F8:B3:B7:3E:FE:88
uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x10, 0x07, 0x34};

// Need this for sending data to peer
esp_now_peer_info_t peerInfo;

// Structure example to send data
typedef struct struct_message{
    float time;
    float depth;
} struct_message;

// Create a struct_message called myData
struct_message myData[SECONDS_UNDERWATER];
struct_message dataPacket[DATA_PER_PACKET];
int seconds = -1;
int currentArray = 0;
bool started = false;
int divesReamining = NUM_DIVES;
bool decending = true;
bool motorsMoving = false;
ESP32Encoder buoyEnc;
bool wifiOn = false;
bool takeDownWifi = false;

void setup(){
    // Init Serial Monitor
    Serial.begin(9600);

    //Set up the motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    //debug light
    pinMode(B_LED, OUTPUT);

    // Set up and initilize encoder
    buoyEnc.attachHalfQuad(ENC1, ENC2);
    buoyEnc.clearCount();

    if(setupPeer()){
        return;
    }
}

bool setupPeer(){
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    wifiOn = true;

    // Init ESP-NOW
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

// Send message via ESP-NOW
void sendData(){
    if(setupPeer()){
        return;
    }

    if(currentArray < ARRAYS_NEEDED){
        // clear old data
        for(int i = 0; i < DATA_PER_PACKET; i++){
            dataPacket[i] = {0.0, 0.0};
        }

        // insert new data
        int dataStart = currentArray * DATA_PER_PACKET;
        for(int i = 0; i < DATA_PER_PACKET; i++){
            if(dataStart + i >= SECONDS_UNDERWATER){
                break;
            }
            dataPacket[i] = myData[dataStart + i];
        }

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataPacket, sizeof(dataPacket));

        if(result == ESP_OK){
            Serial.println("Sent data packet with success");
            currentArray++;
        }
        else{
            Serial.println("Error sending data packet");
        }
    }
}

// callback when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Packet Delivery Success" : "Packet Delivery Failure");
}

//Recieve data as requested
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len){
    struct_message packetData;

    memcpy(&packetData, incomingData, sizeof(packetData));

    if(packetData.time == 1.0){ // (re)start dive sequence
        started = true;
        decending = true;
        buoyEnc.clearCount();
        seconds = -1;
        for(int i = 0; i < SECONDS_UNDERWATER; i++){
            myData[i].time = 0;
            myData[i].depth = 0;
        }
        takeDownWifi = true;
    }
    else if(packetData.time == 2.0){
        if(packetData.depth == -1.0){
            moveFloatUp();
        }
        else if(packetData.depth == 0.0){
            stopFloat();
        }
        else if(packetData.depth == 1.0){
            moveFloatDown();
        }
    }
}

// Start moving the float down
void moveFloatDown(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    motorsMoving = true;
    decending = true;
}

// Start moving the float up
void moveFloatUp(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    motorsMoving = true;
    decending = false;
    delay(1000);                    // this delay may cause issues, consider using a timer instead, but it is needed to ensure the motor starts moving before we check the encoder values
    digitalWrite(B_LED, LOW);
}

// Stop moving the float
void stopFloat(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    motorsMoving = false;
}

//read the data from the depth sensor, normalized to voltage, then to depth in meters
float readDepthSensor(){
    float analogReading = analogRead(DEPTH_PIN);
    float voltage = (analogReading / 4095.0) * 3.3;
    float mAmps = (voltage / 150.0) * 1000;
    float depth = (312.5 * (mAmps - 4)) / 1000;

    return depth;
}

void loop(){
    static long flag = 0;
    static long timer = 0;
    float depth = 0;
    int encoderValue;

    if(takeDownWifi){
        DEFCON1();
        takeDownWifi = false;
        moveFloatDown(); //Done because only time used float should dive
    }

    //Read the depth once per second until the array is full
    timer = millis();
    if(timer - flag >= 1000){
        flag = timer;

        if(started && !wifiOn){
            depth = readDepthSensor();
            seconds++;

            if(seconds < SECONDS_UNDERWATER){
                myData[seconds].time = seconds;
                myData[seconds].depth = depth;
            }
        }
    }

    encoderValue = buoyEnc.getCount();

    //When float reaches the floor, start moving up
    if(started && depth >= DIVE_FLOOR && decending){
        digitalWrite(B_LED, HIGH);
        moveFloatUp();
    }

    //When float reaches the surface, start moving down if a dive remains
    if(started && depth <= DIVE_CEILING && !decending){
        if(--divesReamining > 0){
            moveFloatDown();
        }
        else{
            started = false;
            //stopFloat();
            sendData();
        }
    }

    if(started && motorsMoving && decending && encoderValue <= TARGET_DIVE_PULSES){
        stopFloat();
    }

    if(started && motorsMoving && !decending && encoderValue >= TARGET_SURFACE_PULSES){
        stopFloat();
    }
}