#include <esp_now.h>
#include <WiFi.h>

#define DATA_PER_PACKET 30
#define SECONDS_UNDERWATER 120
#define ARRAYS_NEEDED ((SECONDS_UNDERWATER % DATA_PER_PACKET == 0) ? SECONDS_UNDERWATER / DATA_PER_PACKET : SECONDS_UNDERWATER / DATA_PER_PACKET + 1)
#define BUTTON_START 21
#define PEN_IN 4

//Surface MAC Address: 8C:4F:00:10:07:34
//Water MAC ADDRESS: F8:B3:B7:3E:FE:88
uint8_t broadcastAddress[] = {0xF8, 0xB3, 0xB7, 0x3E, 0xFE, 0x88};

//needed for broadcast purposes
esp_now_peer_info_t peerInfo;

//Structure for data transmission
typedef struct struct_message{
    float time;
    float depth;
} struct_message;

struct_message myData[SECONDS_UNDERWATER];
bool startPressed = false;
bool initialLoop = true;
bool wifiOn = false;
bool takeDownWifi = false;
bool connTakeDownOverride = false;

void setup(){
    //Initialize Serial Monitor
    Serial.begin(9600);

    //BUTTON_START, pin setup
    pinMode(BUTTON_START, INPUT_PULLUP);

    //Delay to allow inital report to print and get output to a new line
    delay(250);
    Serial.println("");
}

bool setupPeer(){
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

//Send singal for dive sequence
void sendStartSignal(){
    if(setupPeer()){
        return;
    }

    struct_message reqStartPacket = {1.0, 1.0};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &reqStartPacket, sizeof(reqStartPacket));

    if(result == ESP_OK){
        Serial.println("Sent start signal with success");
        startPressed = true;
    }
    else{
        Serial.println("Error sending start signal");
    }
}

//send signal to move motor due to potentiometer data
void sendPotentiometerReading(int potentiometerVal){
    if(setupPeer()){
        return;
    }

    struct_message reqStartPacket = {2.0, potentiometerVal};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &reqStartPacket, sizeof(reqStartPacket));

    if (result == ESP_OK){
        Serial.println("Sent potentiometer reading with success");
    }
    else{
        Serial.println("Error sending potentiometer reading: " + String(esp_err_to_name(result)));
    }
}

//Callback when data is sent. Disable peer so we can analong read again
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status){
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Packet Delivery Success" : "Packet Delivery Failure");

    if(connTakeDownOverride){
        connTakeDownOverride = false;
    }
    else{
        takeDownWifi = true;
    }
}

//Recieve data as requested
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len){
    memcpy(&myData, incomingData, sizeof(myData));

    //data is stored line by line as time,depth. This allows the data to imported as proper columns
    for(int i = 0; i < DATA_PER_PACKET; i++){
        Serial.print(myData[i].time);
        Serial.print(",");
        Serial.println(myData[i].depth);
    }
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

    static int debounceTime = 50;

    static bool canRequestStart = true;

    static int potentiometerVal = 0;
    static int potentiometerReading = 0;
    static int potentiometerBuffer = 0;
    static long debouncePotenStart = 0;
    static int potentPrevSend = 0;

    if(takeDownWifi){
        DEFCON1();
        takeDownWifi = false;
    }

    //read raw data
    if(!wifiOn){
        potentiometerReading = readPotentiometer();
    }

    //add vars on same page to start
    if(initialLoop){
        potentiometerVal = potentiometerReading;
        potentiometerBuffer = potentiometerReading;
        potentPrevSend = potentiometerReading;
    }

    //debounce the data
    debounceInt(potentiometerVal, potentiometerReading, potentiometerBuffer, debouncePotenStart, debounceTime);

    //poten send logic
    if(initialLoop || potentPrevSend != potentiometerVal && !startPressed){
        initialLoop = false;
        sendPotentiometerReading(potentiometerVal);
        Serial.println("Potentiometer Value: " + String(potentiometerVal));
        potentPrevSend = potentiometerVal;
    }

    //read the current values of buttons
    currentBUTTON_STARTvalue = digitalRead(BUTTON_START);

    //debounce the inputs
    debounceBool(BUTTON_STARTval, currentBUTTON_STARTvalue, BUTTON_STARTbuffer, debounceSTARTstart, debounceTime);

    //if button for Start is pressed, start dive sequence. CAN BE REPRESSED. THIS WILL RESTART THE PROCEDURE
    if(BUTTON_STARTval == LOW && canRequestStart){ //logical 1
        canRequestStart = false;
        sendPotentiometerReading(0);
        Serial.println("PotentiometerValue: 0");

        //wait for potentiometer to be fully sent before resetting and seding start
        while(!takeDownWifi){
            //wait
        }
        DEFCON1();
        takeDownWifi = false;

        connTakeDownOverride = true;
        sendStartSignal();
    }
    if(BUTTON_STARTval == HIGH){ //logical 0
        canRequestStart = true;
    }
}