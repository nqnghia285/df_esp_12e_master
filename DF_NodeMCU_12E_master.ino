#include <ESP8266WiFi.h>

#include <dhtnew.h>

#include <ArduinoJson.h>

#include <WebSocketsClient.h>
#include <SocketIOclient.h>

#include <Shipper.h>

#include <Hash.h>

// Define const String
#define ON      "on"
#define OFF     "off"
#define CT      "cycle_time"

#define CODE    "CODE001"

// Event
// Server send
#define SSAC    "server-send-ack-connection"
#define SSCM    "server-send-control-machine"
#define SSCD    "server-send-control-device"
#define SSSCT   "server-send-set-cycle-time"
#define SSS     "server-send-script"
#define SSCMOA  "server-send-control-manual-or-auto"

// MCU send
#define MR      "mcu-send-ready"
#define MSACM   "mcu-send-ack-control-machine"
#define MSACD   "mcu-send-ack-control-device"
#define MSASCT  "mcu-send-ack-set-cycle-time"
#define MSAS    "mcu-send-ack-script"
#define MSACMOA "mcu-send-ack-control-manual-or-auto"
#define MSD     "mcu-send-data"

// Message
#define IST     "{\"isSuccess\":true}"
#define ISF     "{\"isSuccess\":false}"

// DHT11 pin
#define dht_pin_D1 5
#define dht_pin_D2 4
#define dht_pin_D3 0

#define USE_SERIAL Serial
//////////////////////////////

int statusOfMachine = -1;
bool eFan = false;
bool bFan = false;
bool heater = false;

// Parameters of script
int temperatureOfScript = 0;

bool isAuto = false;

unsigned long previousTime = 0;
//unsigned long now = 0;
int cycleTime = 5000; // 5s

//////////////////////////////

// Setup dht11 pin
DHTNEW dht1(dht_pin_D1);
DHTNEW dht2(dht_pin_D2);
DHTNEW dht3(dht_pin_D3);
// DHT array
DHTNEW dht[3] = { dht1, dht2, dht3 };

// Read dht i of array dht and return json string "{temperature:?,humidity:?}"
String readDHT(int index) {
  const int capacity = JSON_OBJECT_SIZE(2);
  StaticJsonDocument<capacity> doc;
  
  int chk = dht[index].read();
  if (chk == DHTLIB_OK) {
    doc["temperature"] = dht[index].getTemperature();
    doc["humidity"] = dht[index].getHumidity();
    String output;
    serializeJson(doc, output);
    doc.clear();
    return output; 
  } else {
    return "-1";
  }
}

///////////////////////////////////////////
// Relay pin
#define relay_pin_of_heater         12  // D5
#define relay_pin_of_blow_fan       13  // D6
#define relay_pin_of_exhaust_fan    14  // D7

// Functions control I/O
void turnOn(int pin) {
  digitalWrite(pin, LOW);
}

void turnOff(int pin) {
  digitalWrite(pin, HIGH);
}

void controlDevice(bool stt, int pin){
  if (stt) {
    turnOn(pin);
  } else {
    turnOff(pin);
  }
}

void controlExhaustFan(bool stt) {
  controlDevice(stt, relay_pin_of_exhaust_fan);
}

void controlBlowFan(bool stt) {
  controlDevice(stt, relay_pin_of_blow_fan);
}

void controlHeater(bool stt) {
  controlDevice(stt, relay_pin_of_heater);
}
///////////////////////////////////////////
#define current_pin A0

#define RL    200
#define A     1000
#define Vref  3.3

// Function read current (Ampere)
float measureCurrent(int pin) {
  float Id = -1; // Real current
  int v = 0;  // value read from the sensor
  int v_max = 0;  // store max value here
  uint32_t start_time = millis();
  while ((millis() - start_time) < 100) { // sample for 0.1 sec
    v = analogRead(pin); // read value the from sensor
    if (v > v_max) { // update v_max value
      v_max = v;
    }
  }

  // Loc nhieu o chan A0
  if (v_max < 10) {
    v_max = 0;
  }

  // Url
  float U_RL = (v_max * Vref) / (1023 * 11);

  // Id
  Id = (U_RL * A) / RL;

  // Lam tron 4 chu so thap phan
  int temp = (int)(Id * 10000);
  Id = (temp / 10000.0);

  return Id;
}

//////////////////////////////////
// Read data of sensors
String readSensor() {
  // creat JSON message for Socket.IO (event)
  const int capacity = JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  
  doc["dht1"] = readDHT(0);
  doc["dht2"] = readDHT(1);
  doc["dht3"] = readDHT(2);
  doc["current"] = measureCurrent(current_pin);

  // JSON to String (serializion)
  String result;
  serializeJson(doc, result);

  doc.clear();
  
  return result;
}

//////////////////////////////////
// SSID and password WiFi
const char* ssid = "hw2";
const char* pwd = ".password?";

//const char* ssid = "Redmi";
//const char* pwd = "kalisday";

// Config server side
 char host[] = "192.168.100.54";
//char host[] = "192.168.43.179";
uint16_t port = 5050;
char path[] = "/"; // Socket.IO Base Path

SocketIOclient socket;
Shipper shipper;

// SocketIoEvent
void socketIoEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[SIoC] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[SIoC] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socket.send(sIOtype_CONNECT, path);
            break;
        case sIOtype_EVENT:
            //USE_SERIAL.printf("[SIoC] get event: %s\n", payload);
            
            // Handle message sent from server
            shipper.handleEvent(payload);
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[SIoC] get ack: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[SIoCc] get error: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[SIoC] get binary: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[SIoC] get binary ack: %u\n", length);
            hexdump(payload, length);
            break;
    }
}

/////////////////////////////////////////
// Handle message when received
String handleMessage(const char * payload) {
  String message = String(payload);
  message.replace("\\", "");
//  message.replace(" ", "");
  return message;
}

/////////////////////////////////////////

// Handler event
void serverSendAckConnection(const char * payload, size_t length) {
  USE_SERIAL.print("[connection]Server says: ");
  USE_SERIAL.println(payload);

  const int capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  doc["code"] = CODE;
  doc["status"] = "on";

  String msg;
  serializeJson(doc, msg);

  // MCU send message to Server
  shipper.emit(MR, msg);
  
  doc.clear();
}

void serverSendControlMachine(const char * payload, size_t length) {
  USE_SERIAL.print("[Control Machine]Server says: ");
  USE_SERIAL.println(payload);

  const int capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    statusOfMachine = doc["status"].as<int>();

    // Update status of devices
    if (statusOfMachine == -1 || statusOfMachine == 0) {
      isAuto = false;
      eFan = false;
      bFan = false;
      heater = false;

      controlExhaustFan(false);
      controlBlowFan(false);
      controlHeater(false);
    }

    // MCU send ack to Server
    shipper.emit(MSACM, IST);
  } else {
    // MCU send ack to Server
    shipper.emit(MSACM, ISF);
  }
  
  doc.clear();
}

void serverSendControlDevice(const char * payload, size_t length) {
  USE_SERIAL.print("[Control Device]Server says: ");
  USE_SERIAL.println(payload);

  if (!isAuto) {
    const int capacity = JSON_OBJECT_SIZE(6);
    DynamicJsonDocument doc(capacity);
  
    String message = handleMessage(payload);
  
    DeserializationError err = deserializeJson(doc, message);
    
    if (err == DeserializationError::Ok) {
      
      eFan = doc["e-fan"].as<bool>();
      bFan = doc["b-fan"].as<bool>();
      heater = doc["heater"].as<bool>();
  
      // Control device
      controlExhaustFan(eFan);
      controlBlowFan(bFan);
      controlHeater(heater);
  
      // MCU send ack to Server
      shipper.emit(MSACD, IST);
    } else {
      // MCU send ack to Server
      shipper.emit(MSACD, ISF);
    }
  
    doc.clear();
  }
}

void serverSendSetCycleTime(const char * payload, size_t length) {
  USE_SERIAL.print("[Set Cycle Time]Server says: ");
  USE_SERIAL.println(payload);
  
  const int capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    cycleTime = doc["cycle-time"].as<int>();

    // MCU send ack to Server
    shipper.emit(MSASCT, IST);
  } else {
    // MCU send ack to Server
    shipper.emit(MSASCT, ISF);
  }

  doc.clear();
}

void serverSendScript(const char * payload, size_t length) {
  USE_SERIAL.print("[Script]Server says: ");
  USE_SERIAL.println(payload);

  const int capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    temperatureOfScript = doc["temperature"].as<int>();
    // MCU send ack to Server
    shipper.emit(MSAS, IST);
  } else {
    // MCU send ack to Server
    shipper.emit(MSAS, ISF);
  }

  doc.clear();
}

void serverSendControlManualOrAuto(const char * payload, size_t length) {
  USE_SERIAL.print("[Manual Or Auto]Server says: ");
  USE_SERIAL.println(payload);

  const int capacity = JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    isAuto = doc["is-auto"].as<bool>();
    // MCU send ack to Server
    shipper.emit(MSACMOA, IST);
  } else {
    // MCU send ack to Server
    shipper.emit(MSACMOA, ISF);
  }

  doc.clear();
}
////////////////////////////////////////
// Function read and send data of sensors
void sendSensorData() {
  const int capacity = JSON_OBJECT_SIZE(2) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  JsonObject payload = doc.to<JsonObject>();

  // Add parameter 
  payload["code"] = CODE;

  DynamicJsonDocument params(capacity);
  String result = readSensor();
  
  DeserializationError err = deserializeJson(params, result);
  
  if (err == DeserializationError::Ok) {
    JsonObject objects = params.as<JsonObject>();
    for (JsonObject::iterator it=objects.begin(); it!=objects.end(); ++it) {
      payload[it->key()] = it->value();
    }

    String msg;
    serializeJson(payload, msg);

    // MCU send message to Server
    shipper.emit(MSD, msg);
  } else {
    // MCU send ack to Server
//    shipper.emit(MSD, msg);
  }

  payload.clear();
  params.clear();
  doc.clear();
}

///////////////////////////////////////
// Function handle task follow cycle time
void timerTask(int stt,unsigned long &preTime, int cycleTime) {
  if (stt == 1) {
    unsigned long now = millis();
    
    if (now - preTime > cycleTime) {
      previousTime = now;

      // MCU send sensor data to Server
      sendSensorData();
    }
  }
}

//////////////////////////////////////
void test() {
  // Test relay
  turnOff(relay_pin_of_heater);
  turnOff(relay_pin_of_blow_fan);
  turnOff(relay_pin_of_exhaust_fan);
  delay(500);
  turnOn(relay_pin_of_heater);
  turnOn(relay_pin_of_blow_fan);
  turnOn(relay_pin_of_exhaust_fan);
  delay(500);

  String ms = readSensor();

  shipper.emit(MSD, ms);
  
  USE_SERIAL.println(ms);
}

void setup() {
  USE_SERIAL.begin(115200);
  
  // Setup relay pin
  pinMode(relay_pin_of_heater, OUTPUT);
  pinMode(relay_pin_of_blow_fan, OUTPUT);
  pinMode(relay_pin_of_exhaust_fan, OUTPUT);

  // Set turn off
  controlExhaustFan(false);
  controlBlowFan(false);
  controlHeater(false);

  //Serial.setDebugOutput(true);
  USE_SERIAL.setDebugOutput(true);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for(uint8_t t = 4; t > 0; t--) {
      USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
      USE_SERIAL.flush();
      delay(1000);
  }

  // Connect to wifi 
  USE_SERIAL.print("Connecting to ");
  USE_SERIAL.print(ssid);
  WiFi.begin(ssid, pwd);

  //  Wait esp8266 connected to wifi
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    USE_SERIAL.print('.');
  }

  USE_SERIAL.println();
  USE_SERIAL.print("Connected WiFi: ");
  USE_SERIAL.println(ssid);
  USE_SERIAL.println("IP address(ESP):");
  USE_SERIAL.println(WiFi.localIP());

  // server address, port and URL
  socket.begin(host, port);
  
  // event handler
  socket.onEvent(socketIoEvent);

  //  Setup 'on' listen events
  shipper.on(SSAC, serverSendAckConnection);
  shipper.on(SSCM, serverSendControlMachine);
  shipper.on(SSCD, serverSendControlDevice);
  shipper.on(SSSCT, serverSendSetCycleTime);
  shipper.on(SSS, serverSendScript);
  shipper.on(SSCMOA, serverSendControlManualOrAuto);
}

void loop() {
  // put your main code here, to run repeatedly:
  socket.loop();
  shipper.loop(socket);

  // Send sensor data follow cycle time
  timerTask(statusOfMachine, previousTime, cycleTime);
}
