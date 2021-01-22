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
#define RUNNING "running"

#define CODE    "CODE001"

// Event
// Server send
#define SSAC    "server-send-ack-connection"
#define SSCM    "server-send-control-machine"
#define SSCD    "server-send-control-device"
#define SSSCT   "server-send-set-cycle-time"
#define SSS     "server-send-script"
#define SSFS    "server-send-finish-session"
#define SSCMOA  "server-send-control-manual-or-auto"

// MCU send
#define MR      "mcu-send-ready"
#define MSACM   "mcu-send-ack-control-machine"
#define MSACD   "mcu-send-ack-control-device"
#define MSASCT  "mcu-send-ack-set-cycle-time"
#define MSAS    "mcu-send-ack-script"
#define MSAFS   "mcu-send-ack-finish-session"
#define MSACMOA "mcu-send-ack-control-manual-or-auto"
#define MSD     "mcu-send-data"

// Message
#define IST     "{\"code\":\"CODE001\",\"isSuccess\":true}"
#define ISF     "{\"code\":\"CODE001\",\"isSuccess\":false}"

// DHT11 pin
#define dht_pin_D1 5
#define dht_pin_D2 4
#define dht_pin_D3 0

#define USE_SERIAL Serial
//////////////////////////////

String statusOfMachine = ON;/////////////////////////////////////////////////
bool eFan = false;
bool bFan = false;
bool heater = false;

// Parameters of script
float temperatureOfScript = 31.0;

bool isAuto = false; //////////////////////////////////////////

unsigned long previousTime = 0;
int cycleTime = 3000; // 3s (minimum = 3s)

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
uint16_t port = 5000;
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

  USE_SERIAL.println("Machine connected to Server.");

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
  // {"code":"CODE001","status":on/off/running}

  const int capacity = JSON_OBJECT_SIZE(5);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    String code = doc["code"].as<String>();
    if (code.equals(CODE)) {
      statusOfMachine = doc["status"].as<String>();
      // Update status of devices
        isAuto = false;
        eFan = false;
        bFan = false;
        heater = false;
  
        controlExhaustFan(false);
        controlBlowFan(false);
        controlHeater(false);

        if (statusOfMachine.equals(ON)) {
          USE_SERIAL.println("Machine: ON");
        } else if (statusOfMachine.equals(OFF)) {
          USE_SERIAL.println("Machine: OFF");
        } else if (statusOfMachine.equals(RUNNING)) {
          USE_SERIAL.println("Machine: RUNNING");
        }
  
      // MCU send ack to Server
      shipper.emit(MSACM, IST);
    } else {
      // MCU send ack to Server
      shipper.emit(MSACM, ISF);
    }
  } else {
    // MCU send ack to Server
    shipper.emit(MSACM, ISF);
  }
  
  doc.clear();
}

void serverSendControlDevice(const char * payload, size_t length) {
  USE_SERIAL.print("[Control Device]Server says: ");
  USE_SERIAL.println(payload);
  // {"code":"CODE001","eFan":true/false,"bFan":true/false,"heater":true/false}

  if (statusOfMachine.equals(RUNNING) && !isAuto) {
    const int capacity = JSON_OBJECT_SIZE(9);
    DynamicJsonDocument doc(capacity);
  
    String message = handleMessage(payload);
  
    DeserializationError err = deserializeJson(doc, message);
    
    if (err == DeserializationError::Ok) {
      String code = doc["code"].as<String>();
      if (code.equals(CODE)) {
      
        eFan = doc["eFan"].as<bool>();
        bFan = doc["bFan"].as<bool>();
        heater = doc["heater"].as<bool>();

        USE_SERIAL.print("EFan: ");
        if (eFan) USE_SERIAL.println("ON");
        else USE_SERIAL.println("OFF");
        USE_SERIAL.print("BFan: ");
        if (bFan) USE_SERIAL.println("ON");
        else USE_SERIAL.println("OFF");
        USE_SERIAL.print("Heater: ");
        if (heater) USE_SERIAL.println("ON");
        else USE_SERIAL.println("OFF");
    
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
    } else {
      // MCU send ack to Server
      shipper.emit(MSACD, ISF);
    }
  
    doc.clear();
  } else {
    // MCU send ack to Server
    shipper.emit(MSACD, ISF);
  }
}

void serverSendSetCycleTime(const char * payload, size_t length) {
  USE_SERIAL.print("[Set Cycle Time]Server says: ");
  USE_SERIAL.println(payload);
  // {"code":"CODE001","cycleTime":3000}
  
  const int capacity = JSON_OBJECT_SIZE(5);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    String code = doc["code"].as<String>();
    if (code.equals(CODE)) {
      cycleTime = doc["cycleTime"].as<int>();

      USE_SERIAL.print("Đã thiết lập chu kỳ gửi dữ liệu: ");
      USE_SERIAL.print(cycleTime);
      USE_SERIAL.println(" mili giây");
  
      if (cycleTime < 3000) {
        cycleTime = 3000;
      }
  
  //    Serial.print("Cycle time: ");
  //    Serial.println(cycleTime);
      // MCU send ack to Server
      shipper.emit(MSASCT, IST);
    } else {
      // MCU send ack to Server
      shipper.emit(MSASCT, ISF);
    }
  } else {
    // MCU send ack to Server
    shipper.emit(MSASCT, ISF);
  }

  doc.clear();
}

void serverSendScript(const char * payload, size_t length) {
  USE_SERIAL.print("[Script]Server says: ");
  USE_SERIAL.println(payload);
  // {"code":"CODE001","temperature":32.0}

  const int capacity = JSON_OBJECT_SIZE(7);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    String code = doc["code"].as<String>();
    if (code.equals(CODE)) {
      temperatureOfScript = doc["temperature"].as<float>();

      USE_SERIAL.print("Đã thiết lập nhiệt độ của kịch bản là: ");
      USE_SERIAL.print(temperatureOfScript);
      USE_SERIAL.println(" °C");

      statusOfMachine = RUNNING;
      isAuto = true;
    
      // MCU send ack to Server
      shipper.emit(MSAS, IST);
    } else {
      // MCU send ack to Server
      shipper.emit(MSAS, ISF);
    }
  } else {
    // MCU send ack to Server
    shipper.emit(MSAS, ISF);
  }

  doc.clear();
}

void serverSendControlManualOrAuto(const char * payload, size_t length) { 
  USE_SERIAL.print("[Manual Or Auto]Server says: ");
  USE_SERIAL.println(payload);
  // {"code":"CODE001","isAuto":true/false}
  const int capacity = JSON_OBJECT_SIZE(5);
  DynamicJsonDocument doc(capacity);

  String message = handleMessage(payload);

  DeserializationError err = deserializeJson(doc, message);
  
  if (err == DeserializationError::Ok) {
    String code = doc["code"].as<String>();
    if (code.equals(CODE)) {
      isAuto = doc["isAuto"].as<bool>();

      USE_SERIAL.print("Auto run: ");
      if (isAuto) {
        USE_SERIAL.println("ON");
      } else {
        USE_SERIAL.println("OFF");
      }
      
      // MCU send ack to Server
      shipper.emit(MSACMOA, IST);
    } else {
      // MCU send ack to Server
      shipper.emit(MSACMOA, ISF);
    }
  } else {
    // MCU send ack to Server
    shipper.emit(MSACMOA, ISF);
  }

  doc.clear();
}

void serverSendFinishSession(const char * payload, size_t length) {
  USE_SERIAL.print("[Finish Session]Server says: ");
  USE_SERIAL.println(payload);
  // {"code":"CODE001"}
  
  if (statusOfMachine.equals(RUNNING)) {
    const int capacity = JSON_OBJECT_SIZE(4);
    DynamicJsonDocument doc(capacity);
  
    String message = handleMessage(payload);
  
    DeserializationError err = deserializeJson(doc, message);
    
    if (err == DeserializationError::Ok) {
      String code = doc["code"].as<String>();
      if (code.equals(CODE)) {
        USE_SERIAL.println("Kết thúc quá trình sấy!!!");
        
        isAuto = false;
        eFan = false;
        bFan = false;
        heater = false;
        statusOfMachine = ON;

        controlExhaustFan(eFan);
        controlBlowFan(bFan);
        controlHeater(heater);

        // MCU send ack to Server
        shipper.emit(MSAFS, IST);
      } else {
        // MCU send ack to Server
        shipper.emit(MSAFS, ISF);
      }
    } else {
      // MCU send ack to Server
      shipper.emit(MSAFS, ISF);
    }
  
    doc.clear();
  } else {
    // MCU send ack to Server
    shipper.emit(MSAFS, ISF);
  }
}

////////////////////////////////////////
// Function control device follow script
void controlDeviceFollowScript(float tempAverage) {
  if (isAuto) {
    if (tempAverage > 0.0 && tempAverage > temperatureOfScript) {
      eFan = true;
      bFan = true;
      heater = false;
    } else if (tempAverage > 0.0) {
      eFan = false;
      bFan = false;
      heater = true;
    }

    controlExhaustFan(eFan);
    controlBlowFan(bFan);
    controlHeater(heater);
  }
}

// Function read and send data of sensors + Auto control devices follow script
void sendSensorData() {
  const int capacity = JSON_OBJECT_SIZE(2) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(4) +
                       JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  JsonObject payload = doc.to<JsonObject>();

  DynamicJsonDocument params(capacity);
  String result = readSensor();
  
  DeserializationError err = deserializeJson(params, result);
  
  if (err == DeserializationError::Ok) {
    // Params for calculating average temperature of sensors
    DynamicJsonDocument arr(capacity);
    float tempAverage = 0.0;
    int count = 0;
    /////
    JsonObject objects = params.as<JsonObject>();
    for (JsonObject::iterator it=objects.begin(); it!=objects.end(); ++it) {
      payload[it->key()] = it->value();

      //
      DeserializationError er = deserializeJson(arr, it->value());
      if (er == DeserializationError::Ok) {
        if (arr["temperature"].is<float>()) {
          tempAverage += arr["temperature"].as<float>();
          count++;
        }
      }
    }

    if (count > 0) {
      tempAverage /= count;
      USE_SERIAL.print("Nhiệt độ trung bình: ");
      USE_SERIAL.print(tempAverage);
      USE_SERIAL.println(" °C");
      USE_SERIAL.print("Nhiệt độ của kịch bản: ");
      USE_SERIAL.print(temperatureOfScript);
      USE_SERIAL.println(" °C");
    }

    arr.clear();

    // Auto control device follow script
    controlDeviceFollowScript(tempAverage);

    // Add parameter 
    payload["code"] = CODE;
    payload["eFan"] = eFan;
    payload["bFan"] = bFan;
    payload["heater"] = heater;

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

// Function handle task follow cycle time
void timerTask() {
  if (statusOfMachine.equals(RUNNING)) {
    unsigned long now = millis();
    
    if (now - previousTime > cycleTime) {
      previousTime = now;

      // MCU send sensor data to Server
      sendSensorData();
    }
  }
}
//////////////////////////////////////
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
  shipper.on(SSFS, serverSendFinishSession);
  shipper.on(SSCMOA, serverSendControlManualOrAuto);
}

void loop() {
  // put your main code here, to run repeatedly:
  socket.loop();
  shipper.loop(socket);

  // Send sensor data follow cycle time
  timerTask();
}
