#include <esp_now.h>    // Include ESP-NOW library, needed for wireless communication
#include <WiFi.h>     // Include Wi-Fi library, needed for ESP-NOW
#include <ESP32Encoder.h>   // Include ESP32 Encoder library, needed for reading encoder values

// Pin definitions and constants
#define DEPTH_PIN 25    // Analog pin for depth sensor input
#define IN1 18          // Motor control pin 1 for buoyancy system
#define IN2 19          // Motor control pin 2 for buoyancy system
#define ENC1 17         // Encoder pin 1 for buoyancy system
#define ENC2 4          // Encoder pin 2 for buoyancy system
#define PULSES_PER_REV 2600 // Encoder pulses per revolution
#define TARGET_DIVE_PULSES PULSES_PER_REV * -8  // Target pulses for diving (8 revolutions)
#define TARGET_SURFACE_PULSES 0 // Target pulses for surfacing
#define DATA_PER_PACKET 30  // Number of data points per ESP-NOW packet
#define SECONDS_UNDERWATER 120  // Total seconds to record underwater
#define ARRAYS_NEEDED ((SECONDS_UNDERWATER % DATA_PER_PACKET == 0) ? SECONDS_UNDERWATER / DATA_PER_PACKET : SECONDS_UNDERWATER / DATA_PER_PACKET + 1) // Number of packets needed
#define NUM_DIVES 1     // Number of dives to perform
#define DIVE_FLOOR .5   // was 3; minimum depth to consider "at bottom"
#define DIVE_CEILING 0  // was .666; maximum depth to consider "at surface"
#define B_LED 2         // Debug LED pin
#define DIVE_TIME 10000  // Time in milliseconds to spend diving
#define PAUSE_TIME 20000 // Time in milliseconds to pause at bottom
#define SURFACE_TIME 30000 // Time in milliseconds to spend surfacing (subtract DIVE_TIME from this for time spent surfacing)

//Surface MAC Address: 8C:4F:00:10:07:34
//Water MAC ADDRESS: F8:B3:B7:3E:FE:88
uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x10, 0x07, 0x34};

// Need this for sending data to peer (ESP-NOW)
esp_now_peer_info_t peerInfo;

// Data structure for sending depth data
typedef struct struct_message{
    float time; // Timestamp in seconds
    float depth; // Depth in meters
} struct_message;

// Global variables
struct_message myData[SECONDS_UNDERWATER];  // Array to hold depth data
struct_message dataPacket[DATA_PER_PACKET]; // Packet to send via ESP-NOW
int seconds = -1;                     // Current second index
int currentArray = 0;              // Current data packet index
bool started = false;               // Flag to indicate if dive has started
int divesRemaining = NUM_DIVES;   // Number of dives remaining
bool descending = true;            // Flag to indicate if buoy is descending
bool motorsMoving = false;        // Flag to indicate if motors are active
ESP32Encoder buoyEnc;              // Encoder object for buoyancy system (position tracking)
bool wifiOn = false;            // Flag to indicate if Wi-Fi is on
bool takeDownWifi = false;    // Flag to indicate if Wi-Fi should be taken down

// Function prototypes

// Setup functions
// ----------------
// void setup();
// This function initializes the serial monitor, motor pins, encoder, and sets up ESP-NOW communication.
// Returns: void
void setup(){
    // Init Serial Monitor
    Serial.begin(9600); // Start serial communication at 9600 baud rate

    //Set up the motor pins
    pinMode(IN1, OUTPUT);   // Motor control pin 1
    pinMode(IN2, OUTPUT);   // Motor control pin 2

    //debug light
    pinMode(B_LED, OUTPUT); // On-board LED for debugging

    // Set up and initilize encoder
    buoyEnc.attachHalfQuad(ENC1, ENC2); // Attach encoder to specified pins
    buoyEnc.clearCount();               // Clear encoder count

    if(setupPeer()){    // Setup ESP-NOW peer
        return; // If there was an error, exit setup
    }
}

// bool setupPeer();
// This function sets up the ESP-NOW peer for communication.
// Returns: true if there was an error, false otherwise.
bool setupPeer(){
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);    // Set Wi-Fi mode to Station
    wifiOn = true;         // Update Wi-Fi status flag

    // Init ESP-NOW
    if(esp_now_init() != ESP_OK){   // Initialize ESP-NOW; if it is not ESP_OK, there was an error
        Serial.println("Error initializing ESP-NOW"); // Print error message
        return true;    // Return true to indicate error
    }
    else{
        Serial.println("ESP-NOW initializied successfully");    // Print success message
    }

    // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent); // Register callback for data sent status

    //Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
    esp_now_register_recv_cb(OnDataRecv); // Register callback for data received
  
    // Register peer
    memset(&peerInfo, 0, sizeof(peerInfo)); // Clear peerInfo structure
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);    // Set peer MAC address
    peerInfo.channel = 0;                     // Use current channel
    peerInfo.encrypt = false;                 // No encryption
  
    // Add peer        
    if(esp_now_add_peer(&peerInfo) != ESP_OK){  // Add peer; if not ESP_OK, there was an error
        Serial.println("Failed to add peer");   // Print error message
        return true;    // Return true to indicate error
    }
    else{
        Serial.println("Peer added successfully");  // Print success message
    }

    return false;   // Return false to indicate success
}

// void DEFCON1();
// This function safely deinitializes ESP-NOW and disconnects Wi-Fi.
// Returns: void
void DEFCON1(){
    //Remove Peer
    esp_now_peer_num_t peer_count;  // Structure to hold peer count
    esp_err_t del_result;          // Variable to hold deletion result
    esp_now_get_peer_num(&peer_count);  // Get the number of peers
    if(peer_count.total_num > 0){   // If there are peers to remove, remove the first one
        del_result = esp_now_del_peer(broadcastAddress);    // Delete peer by MAC address
        if(del_result == ESP_OK){        // If deletion was successful
            Serial.println("Peer removed successfully via direct MAC address"); // Print success message
        }
        else{
            Serial.println("Failed to remove peer directly: " + String(del_result));    // Print error message with result code
        }
    }
    else{
        Serial.println("No peers to remove.");  // Print message indicating no peers to remove
    }

    //Clear esp_now to re-init it and have a clean client add
    if(esp_now_deinit() == ESP_OK){  // Deinitialize ESP-NOW; if successful
        Serial.println("ESP-NOW deinitialized successfully");   // Print success message
    } 
    else{
        Serial.println("ESP-NOW was not active or deinit failed");  // Print error message
    }

    //Disconnect WIFI
    WiFi.disconnect();       // Disconnect from Wi-Fi
    WiFi.mode(WIFI_OFF);     // Turn off Wi-Fi

    //Var change
    wifiOn = false;          // Update Wi-Fi status flag
}

// Send message via ESP-NOW
// ----------------
// void sendData();
// This function sends the collected depth data via ESP-NOW in packets.
// Returns: void
void sendData(){ 
    if(setupPeer()){    // Setup ESP-NOW peer; if there was an error, exit function
        return;
    }

    if(currentArray < ARRAYS_NEEDED){   // If there are more data packets to send
        // clear old data
        for(int i = 0; i < DATA_PER_PACKET; i++){   // Clear the data packet
            dataPacket[i] = {0.0, 0.0};
        }

        // insert new data
        int dataStart = currentArray * DATA_PER_PACKET; // Calculate starting index for current packet
        for(int i = 0; i < DATA_PER_PACKET; i++){   // Fill the data packet with depth data
            if(dataStart + i >= SECONDS_UNDERWATER){
                break;
            }
            dataPacket[i] = myData[dataStart + i];
        }

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataPacket, sizeof(dataPacket)); // Send the data packet via ESP-NOW

        if(result == ESP_OK){   // If the send was successful
            Serial.println("Sent data packet with success");    // Print success message
            currentArray++; // Increment the current packet index
        }
        else{               // If there was an error sending the packet
            Serial.println("Error sending data packet");    // Print error message
        }
    }
}

// callback when data is sent
// ----------------
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
// This function is a callback that is called when data is sent via ESP-NOW.
// Parameters:
//   mac_addr - MAC address of the recipient
//   status - Status of the sent data (success or failure)
// Returns: void
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Packet Delivery Success" : "Packet Delivery Failure");
}


//Recieve data as requested
// ----------------
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
// This function is a callback that is called when data is received via ESP-NOW.
// Parameters:
//   mac - MAC address of the sender
//   incomingData - Pointer to the received data
//   len - Length of the received data
// Returns: void
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len){
    struct_message packetData;  // Structure to hold received data

    memcpy(&packetData, incomingData, sizeof(packetData));  // Copy incoming data to packetData structure

    if(packetData.time == 1.0){ // (re)start dive sequence; time = 1.0 indicates start command
        started = true;               // Set started flag to true
        descending = true;         // Set descending flag to true
        buoyEnc.clearCount();    // Clear encoder count
        seconds = -1;            // Reset seconds counter
        currentArray = 0;        // Reset packet index so data isn't sent to an exhausted buffer
        for(int i = 0; i < SECONDS_UNDERWATER; i++){    // Clear depth data array
            myData[i].time = 0;
            myData[i].depth = 0;
        }
        takeDownWifi = true;    // Set flag to take down Wi-Fi
    }
    else if(packetData.time == 2.0){    // manual control of float; time = 2.0 indicates manual control command
        if(packetData.depth == -1.0){   // move float up
            moveFloatUp();
        }
        else if(packetData.depth == 0.0){   // stop float
            stopFloat();
        }
        else if(packetData.depth == 1.0){   // move float down
            moveFloatDown();
        }
    }
}

// Start moving the float down
// ----------------
// void moveFloatDown();
// This function activates the motor to move the float downwards.
// Returns: void
void moveFloatDown(){
    digitalWrite(IN1, LOW);     // Set motor control pin 1 low
    digitalWrite(IN2, HIGH);    // Set motor control pin 2 high
    motorsMoving = true;        // Update motors moving flag
    descending = true;           // Update descending flag
}

// Start moving the float up
// ----------------
// void moveFloatUp();
// This function activates the motor to move the float upwards.
// Returns: void
void moveFloatUp(){
    digitalWrite(IN1, HIGH);    // Set motor control pin 1 high
    digitalWrite(IN2, LOW);    // Set motor control pin 2 low
    motorsMoving = true;        // Update motors moving flag
    descending = false;         // Update descending flag
    delay(1000);          //give time for the motor to start
    digitalWrite(B_LED, LOW);   //turn off debug LED when starting ascent
}

// Stop moving the float
// ----------------
// void stopFloat();
// This function deactivates the motor to stop the float's movement.
// Returns: void
void stopFloat(){
    digitalWrite(IN1, LOW); // Set motor control pin 1 low
    digitalWrite(IN2, LOW); // Set motor control pin 2 low
    motorsMoving = false;     // Update motors moving flag
}

//read the data from the depth sensor, normalized to voltage, then to depth in meters
// ----------------
// float readDepthSensor();
// This function reads the depth sensor and converts the analog reading to depth in meters.
// Returns: Depth in meters as a float
float readDepthSensor(){
    float analogReading = analogRead(DEPTH_PIN);    // Read the analog value from the depth sensor
    float voltage = (analogReading / 4095.0) * 3.3; // Convert analog reading to voltage (ESP32 ADC is 12-bit, max value 4095, reference voltage 3.3V)
    float mAmps = (voltage / 150.0) * 1000;        // Convert voltage to milliamps (sensor output is 150 ohms resistor, so I = V/R; converting to mA)
    float depth = (312.5 * (mAmps - 4)) / 1000;      // Convert milliamps to depth in meters (sensor outputs 4-20mA for 0-5m depth; formula derived from sensor specs)

    return depth;   // Return the calculated depth
}

// Main loop
// ----------------
// void loop();
// This function contains the main loop that continuously reads depth data, controls the buoyancy system, and manages data transmission.
// Returns: void
void loop(){
    static long flag = 0;   // Timer flag for depth reading interval
    static long timer = 0;  // Current timer value
    static float depth = 0;        // Variable to hold depth reading
    int encoderValue;   // Variable to hold encoder value
    static unsigned long diveStartTime = 0;    // Timestamp when dive began (set after WiFi teardown)

    if(takeDownWifi){   //If instructed to take down wifi, do so
        DEFCON1();      //Deinitialize ESP-NOW and disconnect Wi-Fi
        takeDownWifi = false;
        diveStartTime = millis();           // Start timer only after WiFi is fully down
        moveFloatDown();
        Serial.print("Dive start time: ");
        Serial.println(diveStartTime);
    }

    //Read the depth once per second until the array is full
    timer = millis();       // Get the current time in milliseconds
    Serial.println(timer);
    if(timer - flag >= 1000){   // Check if 1 second has passed
        flag = timer;    // Update the timer flag
        // if(started && !wifiOn)
        if(started){ //If a dive is started and wifi is off, turn it on to send data later
            depth = readDepthSensor();  // Read depth sensor
            seconds++;           // Increment seconds counter

            if(seconds < SECONDS_UNDERWATER){   // If within the underwater data collection period
                myData[seconds].time = seconds; // Record the time in seconds
                myData[seconds].depth = depth;  // Record the depth in meters
            }
        }
    }

    encoderValue = buoyEnc.getCount();  // Get the current encoder count
    Serial.println(encoderValue);    // Print pulses count?

    // Use elapsed time from dive start for phase checks.
    // This avoids signed/unsigned issues and is independent of how long
    // the device has been running before the dive is triggered.
    if(started){
        unsigned long elapsed = millis() - diveStartTime;

        if(elapsed < DIVE_TIME){                                    // Phase 1: descending (0-10s)
            digitalWrite(B_LED, HIGH);
            moveFloatDown();
        }
        else if(elapsed >= DIVE_TIME && elapsed < PAUSE_TIME){     // Phase 2: hold at bottom (10-20s)
            stopFloat();
        }
        else if(elapsed >= PAUSE_TIME && elapsed < SURFACE_TIME){  // Phase 3: ascending (20-30s)
            digitalWrite(B_LED, LOW);
            moveFloatUp();
        }
        else{                                                       // Done (30s+)
            started = false;
            stopFloat();
            sendData();
        }
    }

}
