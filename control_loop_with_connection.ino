// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

/*--- Libraries ---*/
//include this to work with analog pins
#include <SPI.h>
#include <Arduino.h>

//included to use flash memory as storage
#include <LittleFS_Mbed_RP2040.h>

//included to add json functionality
//#include <Arduino_JSON.h>

//tank level sensor library
#include <Ultrasonic.h>

//include to store data in the arduino nano
#include <Arduino_JSON.h>

//include to use button
#include <Bounce2.h>

// C99 libraries.
#include <cstdbool>
#include <cstdlib>
#include <cstring>
#include <time.h>

// Libraries for SSL client, MQTT client, and WiFi connection.
#include <ArduinoBearSSL.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>

// Libraries for SAS token generation.
#include <ECCX08.h>

// Azure IoT SDK for C includes.
#include <az_core.h>
#include <az_iot.h>

// Sample header.
#include "iot_configs.h"

// Logging
#include "SerialLogger.h"

/*--- Macros ---*/
#define BUFFER_LENGTH_MQTT_CLIENT_ID 256
#define BUFFER_LENGTH_MQTT_PASSWORD 256
#define BUFFER_LENGTH_MQTT_TOPIC 128
#define BUFFER_LENGTH_MQTT_USERNAME 512
#define BUFFER_LENGTH_SAS 32
#define BUFFER_LENGTH_SAS_ENCODED_SIGNED_SIGNATURE 64
#define BUFFER_LENGTH_SAS_SIGNATURE 512
#define BUFFER_LENGTH_DATETIME_STRING 256

//#define LED_PIN 2  // High on error. Briefly high for each successful send.

// Time and Time Zone.
#define SECS_PER_MIN 60
#define SECS_PER_HOUR (SECS_PER_MIN * 60)
#define GMT_OFFSET_SECS (IOT_CONFIG_DAYLIGHT_SAVINGS ? ((IOT_CONFIG_TIME_ZONE + IOT_CONFIG_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * SECS_PER_HOUR) : (IOT_CONFIG_TIME_ZONE * SECS_PER_HOUR))

// Exit into infinite loop
#define EXIT_LOOP(condition, errorMessage) \
  do \ 
  { \
      if (condition) { \
        Logger.Error(errorMessage); \
        while (1) \
          ; \
      } \
    } \
  while (0)


/*--- Sample static variables --*/
// Clients for WiFi connection, SSL, MQTT, and Azure IoT SDK for C.
static WiFiClient wiFiClient;
static BearSSLClient bearSSLClient(wiFiClient);
static MqttClient mqttClient(bearSSLClient);
static az_iot_hub_client azIoTHubClient;

// MQTT variables.
static char mqttClientId[BUFFER_LENGTH_MQTT_CLIENT_ID];
static char mqttUsername[BUFFER_LENGTH_MQTT_USERNAME];
static char mqttPassword[BUFFER_LENGTH_MQTT_PASSWORD];

// Telemetry variables.
static char telemetryTopic[BUFFER_LENGTH_MQTT_TOPIC];
static unsigned long telemetryNextSendTimeMs;
static String telemetryPayload;
String zoneInfo;
static uint32_t telemetrySendCount;

/*--- Functions ---*/
// Initialization and connection functions.
void connectToWiFi();
void initializeAzureIoTHubClient();
void initializeMQTTClient();
void connectMQTTClientToAzureIoTHub();

// Telemetry and message-callback functions.
void onMessageReceived(int messageSize);
static void sendTelemetry();
static char* generateTelemetry();

// SAS Token related functions.
static void generateMQTTPassword();
static void generateSASBase64EncodedSignedSignature(
  uint8_t const* sasSignature, size_t const sasSignatureSize,
  uint8_t* encodedSignedSignature, size_t encodedSignedSignatureSize,
  size_t* encodedSignedSignatureLength);
static uint64_t getSASTokenExpirationTime(uint32_t minutes);

// Time and Error functions.
static unsigned long getTime();
static String getFormattedDateTime(unsigned long epochTimeInSeconds);
static String mqttErrorCodeName(int errorCode);

/*---------------------------*/
/*    Main code execution    */
/*---------------------------*/

/*
 * setup:
 * Initialization and connection of serial communication, WiFi client, Azure IoT SDK for C client, 
 * and MQTT client.
 */


//define variables used for reading and reporting about tank levels
int tankLevel;
int tankThreshold = 1;
const int tankSensorHeight = 4;  //height of sensor from bottom of tank in cm
const int tankTriggerPin = 2;
const int tankEchoPin = 3;
Ultrasonic ultrasonic(2, 3);  //object used to easily read the moisture levels


bool noZonesActive = true;  // global variable used in void loop and checkAndWater used to check that all zones aren't running to avoid running more than one zone at once

class WateringZone {  //object used to define each of the 7 zones of the garden
  //public constructor: defines watering zone object based off of input variables
public:
  WateringZone(int zoneNumber, int relayPin, bool zoneInstalled, int moistureThreshold, int wateringTime, int minWateringGap, int maxWateringGap, int lowCalValue, int highCalValue) {

    _zoneNumber = zoneNumber;  //zone number/pin for solenoid
    _relayPin = relayPin;
    _zoneInstalled = zoneInstalled;
    _moistureThreshold = moistureThreshold;  //lower limit before watering is turned on
    _wateringTime = wateringTime;            //length of watering time
    _minWateringGap = minWateringGap;        // defines time interval between the zone being watered again
    _maxWateringGap = maxWateringGap;        //maximum amount of time, before it must be watered
    _lastWateringTime = 0;                   //default time, gets changed after watered
    _moistureValue;                   //default value gets changed after analog value is read
    _startedWateringTime;                    //comparison timestamp, set to millis()/1000 when the pump is turned on
    _isWatering = false;                     // value is change to true when solenoid/pump are on
    _lowCalValue = lowCalValue;
    _highCalValue = highCalValue;

    //calibration button initialization, not needed atm
    // calButton.attach(2, INPUT);
    // calButton.interval(50);
  }
  // function that calibrates the moisture sensor values, and returns the moisture values on scale from 0-100
          // int lowAndHigh() {
          //   // _doneCalibrating = true;     //currently set to true to avoid the calibration step
          //   // while (!_doneCalibrating) {  //continually tells user on serial monitor to press calibration button to begin the mode
          //   //   Serial.println("Press the button to start calibrating");
          //   //   calButton.update();
          //   //   delay(1000);

          //   //   if (calButton.fell()) {  //once the button has been pressed, prompt the user to submerge the sensor, then press the set/calibration button
          //   //     while (!_lowValueSet) {
          //   //       readMoisturePin();
          //   //       Serial.println("Press the button after submerging the sensor" + String(_moistureValue));
          //   //       calButton.update();
          //   //       delay(1000);

          //   //       if (calButton.fell()) {
          //   //         readMoisturePin();
          //   //         _lowValue = _moistureValue;
          //   //         Serial.println("Low value set: " + String(_lowValue));
          //   //         _lowValueSet = true;
          //   //       }
          //   //     }

          //   //     while (!_highValueSet) {
          //   //       readMoisturePin();
          //   //       Serial.println("Press the button after drying off the sensor" + String(_moistureValue));
          //   //       calButton.update();
          //   //       delay(1000);

          //   //       if (calButton.fell()) {
          //   //         readMoisturePin();
          //   //         _highValue = _moistureValue;
          //   //         Serial.println("High value set: " + String(_highValue));
          //   //         _highValueSet = true;
          //   //       }
          //   //     }

          //   //     _doneCalibrating = true;
          //   //   }
          //   // }

          //   // Use the map function to return a mapped value
          //   readMoisturePin();
          //   int mappedValue = map(_moistureValue, _lowValue, _highValue, 100, 0);
          //   return mappedValue;
          // }
  // checks the time period between now and last time the zone was watered as well as the moisture value
  void readMoisturePin() {  //Can't store A4-A7 as a variable, so its necessary to analogRead by zone number
    int moistureReading;
    if (_zoneNumber == 1) {
      moistureReading = analogRead(A0);
    } else if (_zoneNumber == 2) {
      moistureReading = analogRead(A1);
    } else if (_zoneNumber == 3) {
      moistureReading = analogRead(A2);
    } else if (_zoneNumber == 4) {
      moistureReading = analogRead(A3);
    } else if (_zoneNumber == 5) {
      moistureReading = analogRead(A4);
    } else if (_zoneNumber == 6) {
      moistureReading = analogRead(A6);
    } else {
      moistureReading = analogRead(A7);
    }
       _moistureValue = map(moistureReading, _lowCalValue, _highCalValue, 100, 0 );
  }

  void checkAndWater() {
    // int currentMillis = millis() / 1000; //current time in seconds
    // int moistureValue; //moisture value variable
    // //arduino nano is annoying so we can't store the analog pins a4-a7 as integers
    // readMoisturePin();

    // if (currentMillis - _lastWateringTime >= _minWateringGap) {     // if it has been long enough
    //   if (_moistureValue < _moistureThreshold || currentMillis - _lastWateringTime >= _maxWateringGap) { //and the moisture value is below the threshold or its been long enough
    //     digitalWrite(_zoneNumber, HIGH); //turn zone on
    //     delay(_wateringTime); //wait the user specified amount of time
    //     digitalWrite(_zoneNumber, LOW); //turn the pin off once the time is up
    //     _lastWateringTime = currentMillis; //reset the time of last watering to now
    //   }
    // }
    int pumpPin = 12;
    int currentMillis = millis() / 1000;                                          //current time variable for comparison in seconds
    readMoisturePin();                                                //update the zones current moisture value by reading sensor
    if (currentMillis - _lastWateringTime >= _minWateringGap) {              // if it has been long enough since it was last watered and the tank isn't too low
      if (_moistureValue < _moistureThreshold || currentMillis - _lastWateringTime >= _maxWateringGap) {  //and the moisture value is below the threshold or its been long enough
        if (_isWatering == false) {                                                                       //conditions have been met so turn the pump on if its not already on
          _isWatering = true;
          digitalWrite(_relayPin, HIGH);
          delay(500);  //to avoid malfunction
          digitalWrite(pumpPin, HIGH);
          _startedWateringTime = millis() / 1000;
        }
      }
    }
    if (_isWatering && millis() / 1000 - _startedWateringTime > _wateringTime) {  //turns off pump/valve if enough water has been sent
      digitalWrite(pumpPin, LOW);
      delay(500);  //to avoid a malfunction
      digitalWrite(_relayPin, LOW);
      _lastWateringTime = millis() / 1000;  //record the most recent time the plant was watered
      _isWatering = false;
      noZonesActive = true;  //only one zone will work at a time, so set it to true after turning off the watering
    }
  }
  //get and set methods for relevant zone variables
  //added 11.15
  bool getIsWatering() {
    return _isWatering;
  }
  int getRelayPin() {
    return _zoneNumber;
  }
  int getMoistureThreshold() {
    return _moistureThreshold;
  }

  void setMoistureThreshold(int moistureThreshold) {
    _moistureThreshold = moistureThreshold;
  }

  void setZoneInstalled(bool zoneInstalled){
    _zoneInstalled = zoneInstalled;
  }
  // Getter and setter methods for wateringTime
  int getWateringTime() {
    return _wateringTime;
  }

  void setWateringTime(int wateringTime) {
    _wateringTime = wateringTime;
  }

  // Getter and setter methods for minWateringGap
  int getMinWateringGap() {
    return _minWateringGap;
  }

  void setMinWateringGap(int minWateringGap) {
    _minWateringGap = minWateringGap;
  }

  // Getter and setter methods for maxWateringGap
  int getMaxWateringGap() {
    return _maxWateringGap;
  }

  void setMaxWateringGap(int maxWateringGap) {
    _maxWateringGap = maxWateringGap;
  }
  // getting the analog moisture value
  int getMoistureValue() {
    return _moistureValue;
  }
  void setMoistureValue(int moistureValue) {
    _moistureValue = moistureValue;
  }
  // zone info
  String getZoneInfo() { //edited to only give necessary info to azure api
    String zoneInfo;
    zoneInfo = "Zone: " + String(_zoneNumber) + "; ";
    zoneInfo += "Moisture Value: " + String(_moistureValue);
    zoneInfo += "; Is watering: " + String(_isWatering);
    zoneInfo += "Last Watering Time: " + String(_lastWateringTime) + "; ";
    return zoneInfo;
   // zoneInfo += "Relay Pin: " + String(_relayPin) + "; ";
   // zoneInfo += "Zone is Installed: " + String(_zoneInstalled);
   // zoneInfo += "Moisture Threshold: " + String(_moistureThreshold) + "; ";
  //zoneInfo += "Watering Time: " + String(_wateringTime) + "; ";
   // zoneInfo += "Min Watering Gap: " + String(_minWateringGap) + "; ";
   // zoneInfo += "Max Watering Gap: " + String(_maxWateringGap) + "; ";
   //return zoneInfo + "; Current millis in seconds: " + String(millis() / 1000);
  }
  
  String getJSONZoneInfo() {

    JSONVar zoneData; //initialize json object for this zone

    //add zone data with key
    zoneData["moistureValue"] = _moistureValue;
    zoneData["zoneNumber"] = _zoneNumber;
    zoneData["isWatering"] = _isWatering;
    zoneData["lastWateringTime"] = _lastWateringTime;
    
    return zoneData;
  }

  // private constructor creates object with these varaibles
private:
  int _zoneNumber;
  int _relayPin;
  bool _zoneInstalled;
  int _moistureThreshold;
  int _wateringTime;
  int _minWateringGap;
  int _maxWateringGap;
  int _lastWateringTime;
  int _moistureValue;
  int _startedWateringTime;
  bool _isWatering;
  int _lowCalValue;
  int _highCalValue;

  //button setup
    // bool _doneCalibrating = false;
    // bool _lowValueSet = false;
    // int _lowValue = 0;
    // bool _highValueSet = false;
    // int _highValue = 0;
    // Bounce calButton;
};


// bool writeStringToLittleFS(const char* fileName, const String& data) {
//   FILE* writeFile = fopen(fileName, "w");

//   if (writeFile) {
//     size_t bytesWritten = fwrite((const uint8_t*)data.c_str(), 1, data.length(), writeFile);
//     fclose(writeFile);
//     return bytesWritten == data.length();
//   }

//   return false;
// }

// String readFromLittleFS(const char* filePath) {
//   String fileContent = "";

//   FILE* readFile = fopen(filePath, "r");
//   if (readFile) {
//     fseek(readFile, 0, SEEK_END);
//     long fileSize = ftell(readFile);
//     fseek(readFile, 0, SEEK_SET);

//     char* buffer = (char*)malloc(fileSize + 1);
//     if (buffer) {
//       fread(buffer, 1, fileSize, readFile);
//       buffer[fileSize] = '\0';  // Correct null termination

//       fclose(readFile);

//       fileContent = String(buffer);
//       free(buffer);
//     } else {
//       Serial.println("Failed to allocate memory");
//     }
//   } else {
//     Serial.println("Failed to open file");
//   }
//   return fileContent;
// }

//DEFAULT VALUES

// //grab stored value
// LittleFS_MBED* myFS;
// const char* filePath = "/internalstorage.txt";
// //Write default values to files, will get c
// //String messageReceived = readFromLittleFS(filePath);
// String messageRecieved = "";
// //now parse stored text
// String storedZoneData[7];
// int semiIndex;


// String removeSpaces(String str) {
//   str.replace(" ", "");  // Replace spaces with an empty string
//   return str;
// }
// if (messageReceived == "") {
//   messageReceived = "1, 500, 10 , 20, 30; 2, 500, 10 , 20, 30; 3, 500, 10 , 20, 30;\
//                      4, 500, 10 , 20, 30; 5, 500, 10 , 20, 30; 6, 500, 10 , 20, 30;\
//                      7, 500, 10 , 20, 30";
// }
// // Remove spaces from the received message
// messageReceived = removeSpaces(messageReceived);

// // Parse received message into storedZoneData
// for (int i = 0; i < 7; i++) {
//   semiIndex = messageReceived.indexOf(';');

//   if (semiIndex != -1) {
//     storedZoneData[i] = messageReceived.substring(0, semiIndex);
//     messageReceived = messageReceived.substring(semiIndex + 1);
//   } else {
//     break;
//   }
// }

// int zoneValues[7][5];
// int commaIndex;

// // Parse storedZoneData into zoneValues
// for (int i = 0; i < 7; i++) {
//   for (int j = 0; j < 5; j++) {
//     commaIndex = storedZoneData[i].indexOf(",");

//     if (commaIndex != -1) {
//       String value = storedZoneData[i].substring(0, commaIndex);
//       value.trim();  // Remove leading/trailing spaces (if any)
//       zoneValues[i][j] = atoi(value.c_str());
//       storedZoneData[i] = storedZoneData[i].substring(commaIndex + 1);
//     } else {
//       break;
//     }
//   }
// }
//should create: addUserParameters()
//default values
bool defInstallStatus=1;
int defMoistureThreshold = 500;
int defWateringTime = 10;
int defMinWateringGap = 20;
int defMaxWateringGap = 30;
int defLowCalValue = 250;
int defHighCalValue = 775;


//psuedocode for eventually filling the waterzones with flash memory
//read memory location
// break the read memory up into 7 objects
//set each watering zone o
WateringZone zoneOne = WateringZone(1, 11, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneTwo = WateringZone(2, 10, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneThree = WateringZone(3, 8, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneFour = WateringZone(4, 7, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneFive = WateringZone(5, 6, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneSix = WateringZone(6, 5, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneSeven = WateringZone(7, 4, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);



WateringZone zoneArray[7] = {zoneOne, zoneTwo, zoneThree, zoneFour, zoneFive, zoneSix, zoneSeven};

void setup() {  //code that only runs once

  while (!Serial);
  Serial.begin(SERIAL_LOGGER_BAUD_RATE);
 // pinMode(LED_PIN, OUTPUT);

 // digitalWrite(LED_PIN, HIGH);

  //code below doesn't seem to be necessary, will need to be tested-11.19.2023
  //New code intializes iothub/mqtt if the wifi login works, and if it doesnt, try 10 times for a total of 30 seconds
  // for(int i = 1; i < 11; i++){
  //   connectToWiFi();
  //   if(WiFi.status() == WL_CONNECTED){
  //     initializeAzureIoTHubClient();
  //     initializeMQTTClient();
  //     connectMQTTClientToAzureIoTHub();
  //     i=12;
  //   }
  //   else{
  //     delay(3000); //wait 3 seconds between attempts
  //     Serial.println("Attempt: " + String(i) + " of setup");
  //   }
  // }

  //Template initialization
  // connectToWiFi();
  // initializeAzureIoTHubClient();
  // initializeMQTTClient();
  // connectMQTTClientToAzureIoTHub();

  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    initializeAzureIoTHubClient();
    initializeMQTTClient();
    connectMQTTClientToAzureIoTHub();
  }

  //digitalWrite(LED_PIN, LOW);
  telemetryNextSendTimeMs = 0;

  //initializing the digital pins used to control the relay
  for (int i = 4; i < 13; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  // myFS = new LittleFS_MBED();

  // if (!myFS->init()) {
  //   Serial.println("LITTLEFS Mouhnt Failed");
  //   return;
  // }
}

//old code that created 7 zones
//  for(int solenoidPin = 1; solenoidPin < 8; solenoidPin++){
//    pinMode(solenoidPin, OUTPUT);
//    zones[solenoidPin - 1] = WateringZone(solenoidPin, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap);
//  }

//   //zones only has one watering zone object
//   for(int solenoidPin = 1; solenoidPin < 2; solenoidPin++){
//   pinMode(solenoidPin, OUTPUT); //set pin mode
//   zones[solenoidPin - 1] = WateringZone(solenoidPin, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap);
// }

//}

/*
 * loop:
 * Check for connection and reconnect if necessary.
 * Send Telemetry and receive messages.
 */
void loop() {
  // code that runs repeatedly until disconnected from power
  if (WiFi.status() != WL_CONNECTED) {  //tries to connect to the cloud if not connected
    connectToWiFi();
    if(WiFi.status() == WL_CONNECTED) { // IF CONNECTION SUCCEEDS, BUT IT WAS DISCONNECTED AT SOME POINT, THIS INITIALIZES AZURE THINGS
    initializeAzureIoTHubClient();
    initializeMQTTClient();
    connectMQTTClientToAzureIoTHub();
    }
  }
  else{ //if its already connected
      if (millis() > telemetryNextSendTimeMs)  //if its been long enough since the last message was sent
    {
      // Check for MQTT Client connection to Azure IoT hub. Reconnect if needed.
      if (!mqttClient.connected())  
      {
        connectMQTTClientToAzureIoTHub();
      }
      sendTelemetry(); //triggers telemetryPayload to be filled with zone info, then sends it
      telemetryNextSendTimeMs = millis() + IOT_CONFIG_TELEMETRY_FREQUENCY_MS;
    }

    // MQTT loop must be called to process Telemetry and Cloud-to-Device (C2D) messages.
    mqttClient.poll();
    //delay(50); removed to stop interrupts from disrupting system function
  }

  // Telemetry
  //old template code
  // if (WiFi.status() != WL_CONNECTED) 
  //   {
  //     connectToWiFi();
  //   }

  //   // Telemetry
  //   if (millis() > telemetryNextSendTimeMs) 
  //   {
  //     // Check for MQTT Client connection to Azure IoT hub. Reconnect if needed.
  //     if (!mqttClient.connected()) 
  //     {
  //       connectMQTTClientToAzureIoTHub();
  //     }

  //     sendTelemetry();
  //     telemetryNextSendTimeMs = millis() + IOT_CONFIG_TELEMETRY_FREQUENCY_MS;
  //   }

  //   // MQTT loop must be called to process Telemetry and Cloud-to-Device (C2D) messages.
  //   mqttClient.poll();
  //   delay(50);

  //new code as of 11.15, only allows one solenoid to open at a time
  tankLevel = ultrasonic.read() - tankSensorHeight;  //updates tank level adjusting for height of sensor

  if(tankLevel < tankThreshold){
    for(int i = 0; i < 7; i++){
      zoneArray[i].readMoisturePin();
    }
  }
  else{
    //checks if a zone is already watering or not
    for (int i = 0; i < 7; i++) {  //if any of the zones are already running
      if (zoneArray[i].getIsWatering()) {
        noZonesActive = false;
        zoneArray[i].checkAndWater();  //need to check if its been long enough if the pump is currently running
        i = 8;
      }
    }
    if (noZonesActive) {  // only runs if no zones are currently being watered
      for (int j = 0; j < 7; j++) {
        zoneArray[j].checkAndWater();        //check to see if its time to water
        if (zoneArray[j].getIsWatering()) {  //if this causes it to start watering, no need to check other zones
          j = 9;
        }
      }
    }
  }
}


//writing to memory function

/*-----------------------------------------------*/
/*    Initialization and connection functions    */
/*-----------------------------------------------*/

/*
 * connectToWifi:
 * The WiFi client connects, using the provided SSID and password.
 * The WiFi client synchronizes the time on the device. 
 */
void connectToWiFi() {

  //code has been modified so that if it fails, it doesn't get stuck
  Logger.Info("Attempting to connect to WIFI SSID: " + String(IOT_CONFIG_WIFI_SSID));

  WiFi.begin(IOT_CONFIG_WIFI_SSID, IOT_CONFIG_WIFI_PASSWORD);  // attempt to connect to wifi

  if (WiFi.status() == WL_CONNECTED) {  //if coonection attempt was successful
    Serial.println();

    Logger.Info("WiFi connected, IP address: " + String(WiFi.localIP()) + ", Strength (dBm): " + WiFi.RSSI());
    Logger.Info("Syncing time.");

    while (getTime() == 0) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();

    Logger.Info("Time synced!");
  } 
  else {
    Serial.println("Connection Attempt Failed");
  }





  //continual reconnection code
  // int lastAttemptTime = millis() / 1000;
  // while(lastAttemptTime - millis() > 30 && WiFi.status){ //if its been less than 20 seconds and wifi still hasn't worked yet
  //     Serial.println("Last attempt failed, trying again: ");
  //     WiFi.begin(IOT_CONFIG_WIFI_SSID, IOT_CONFIG_WIFI_PASSWORD);
  //     delay(5000); //wait 5 seconds before trying again

  // }
  // DOESN'T ALLOW CODE TO FUNCTION WITHOUT WIFI
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   Serial.print(".");
  //   delay(IOT_CONFIG_WIFI_CONNECT_RETRY_MS);
  // }
  // Serial.println();

  // Logger.Info("WiFi connected, IP address: " + String(WiFi.localIP()) + ", Strength (dBm): " + WiFi.RSSI());
  // Logger.Info("Syncing time.");

  // while (getTime() == 0)
  // {
  //   Serial.print(".");
  //   delay(500);
  // }
  // Serial.println();

  // Logger.Info("Time synced!");
}

/*
 * initializeAzureIoTHubClient:
 * The Azure IoT SDK for C client uses the provided hostname, device id, and user agent.
 */
void initializeAzureIoTHubClient() {
  Logger.Info("Initializing Azure IoT Hub client.");

  az_span hostname = AZ_SPAN_FROM_STR(IOT_CONFIG_IOTHUB_FQDN);
  az_span deviceId = AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_ID);

  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(IOT_CONFIG_AZURE_SDK_CLIENT_USER_AGENT);

  int result = az_iot_hub_client_init(&azIoTHubClient, hostname, deviceId, &options);

  EXIT_LOOP(az_result_failed(result), "Failed to initialize Azure IoT Hub client. Return code: " + result);

  Logger.Info("Azure IoT Hub hostname: " + String(IOT_CONFIG_IOTHUB_FQDN));
  Logger.Info("Azure IoT Hub client initialized.");
}

/*
 * initializeMQTTClient:
 * The MQTT client uses the client id and username from the Azure IoT SDK for C client.
 * The MQTT client uses the generated password (the SAS token).
 */
void initializeMQTTClient() {
  Logger.Info("Initializing MQTT client.");

  int result;

  result = az_iot_hub_client_get_client_id(
    &azIoTHubClient, mqttClientId, sizeof(mqttClientId), NULL);
  EXIT_LOOP(az_result_failed(result), "Failed to get MQTT client ID. Return code: " + result);

  result = az_iot_hub_client_get_user_name(
    &azIoTHubClient, mqttUsername, sizeof(mqttUsername), NULL);
  EXIT_LOOP(az_result_failed(result), "Failed to get MQTT username. Return code: " + result);

  generateMQTTPassword();  // SAS Token

  mqttClient.setId(mqttClientId);
  mqttClient.setUsernamePassword(mqttUsername, mqttPassword);
  mqttClient.onMessage(onMessageReceived);  // Set callback for C2D messages

  Logger.Info("Client ID: " + String(mqttClientId));
  Logger.Info("Username: " + String(mqttUsername));

  Logger.Info("MQTT client initialized.");
}

/*
 * connectMQTTClientToAzureIoTHub:
 * The SSL library sets a callback to validate the server certificate.
 * The MQTT client connects to the provided hostname. The port is pre-set.
 * The MQTT client subscribes to the Cloud to Device (C2D) topic to receive messages.
 */
void connectMQTTClientToAzureIoTHub() {
  Logger.Info("Connecting to Azure IoT Hub.");

  // Set a callback to get the current time used to validate the server certificate.
  ArduinoBearSSL.onGetTime(getTime);

  while (!mqttClient.connect(IOT_CONFIG_IOTHUB_FQDN, AZ_IOT_DEFAULT_MQTT_CONNECT_PORT)) {
    int code = mqttClient.connectError();
    Logger.Error("Cannot connect to Azure IoT Hub. Reason: " + mqttErrorCodeName(code) + ", Code: " + code);
    delay(5000);
  }

  Logger.Info("Connected to your Azure IoT Hub!");

  mqttClient.subscribe(AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC);

  Logger.Info("Subscribed to MQTT topic: " + String(AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC));
}

/*------------------------------------------------*/
/*    Telemetry and message-callback functions    */
/*------------------------------------------------*/

/*
 * onMessageReceived:
 * The function called when device receives a message on the subscribed C2D topic.
 * Callback function signature is defined by the ArduinoMQTTClient library.
 * Message received is printed to the terminal.
 */
void onMessageReceived(int messageSize) {
  //attempt at parsing string

  String recievedMessage = "";
  while (mqttClient.available()) //fills recievedMessage, one char at a time
  {
    char recievedChar = (char)mqttClient.read();
    recievedMessage += recievedChar;
   // Serial.print((char)mqttClient.read());
  }
  Serial.println(recievedMessage);

  JSONVar zonesData = JSON.parse(recievedMessage); //parses the json messge

  for (int i = 0; i < 7; i++){
    JSONVar zoneData = zonesData[i];
    //extract zone specific data
    zoneArray[i].setZoneInstalled(zoneData["zoneInstalled"]);
    zoneArray[i].setWateringTime(zoneData["wateringTime"]);
    zoneArray[i].setMinWateringGap(zoneData["minWateringGap"]);
    zoneArray[i].setMaxWateringGap(zoneData["maxWateringGap"]);
    zoneArray[i].setMoistureThreshold(zoneData["moistureThreshold"]);
  }
  //template code
  // Logger.Info("Message received: Topic: " + mqttClient.messageTopic() + ", Length: " + messageSize);
  // Logger.Info("Message: ");

  // while (mqttClient.available())
  // {
  //   Serial.print((char)mqttClient.read());
  // }
  // Serial.println();
}

/*
 * sendTelemetry:
 * The Azure IoT SDK for C client creates the MQTT topic to publish a telemetry message.
 * The MQTT client creates and sends the telemetry mesage on the topic.
 */
static void sendTelemetry() {
  //digitalWrite(LED_PIN, HIGH);
  Logger.Info("Arduino Nano RP2040 Connect sending telemetry . . . ");

  int result = az_iot_hub_client_telemetry_get_publish_topic(
    &azIoTHubClient, NULL, telemetryTopic, sizeof(telemetryTopic), NULL);
  EXIT_LOOP(az_result_failed(result), "Failed to get telemetry publish topic. Return code: " + result);

  mqttClient.beginMessage(telemetryTopic);
  mqttClient.print(generateTelemetry());
  mqttClient.endMessage();

  Logger.Info("Telemetry sent.");
  delay(100);
  //digitalWrite(LED_PIN, LOW);
}

/*
 * generateTelemetry:
 * Simulated telemetry.  
 * In your application, this function should retrieve real telemetry data from the device and format
 * it as needed.
 */
static char* generateTelemetry() {
  //template code:
  //telemetryPayload =  String("{ \"msgCount\": ") + telemetrySendCount + " }"; //prints out the message count
  //telemetrySendCount++; //upticks message count
  //test code using wateringzone message function

  //inefficient payload definition: telemetryPayload = zoneArray[0].getZoneInfo() + "\n" + zoneArray[1].getZoneInfo() + "\n" + zoneArray[2].getZoneInfo();
  Serial.println("Tries to generate");
  zoneInfo =  String("{ \"msgCount\": ") + telemetrySendCount + " }"; //prints out the message count
  telemetrySendCount++;
  Serial.println("makes it to 4 loop");
  for(int i = 0; i < 7; i++){//fill the telemetry message with zone info
    Serial.println(zoneInfo);
    zoneInfo = zoneInfo +  zoneArray[i].getZoneInfo() + "\n";
  }
  telemetryPayload = zoneInfo;
  Serial.println((char*)telemetryPayload.c_str());
  return (char*)telemetryPayload.c_str();
}

/*************************************/
/*    SAS Token related functions    */
/*************************************/

/*
 * generateMQTTPassword:
 * The MQTT password is the generated SAS token. The process is:
 *    1. Get the SAS token expiration time from the provided value. (Default 60 minutes).
 *    2. Azure IoT SDK for C creates the SAS signature from this expiration time.
 *    3. Sign and encode the SAS signature.
 *    4. Azure IoT SDK for C creates the MQTT Password from the expiration time and the encoded,
 *       signed SAS signature.
 */
static void generateMQTTPassword() {
  int result;

  uint64_t sasTokenDuration = 0;
  uint8_t signature[BUFFER_LENGTH_SAS_SIGNATURE] = { 0 };
  az_span signatureAzSpan = AZ_SPAN_FROM_BUFFER(signature);
  uint8_t encodedSignedSignature[BUFFER_LENGTH_SAS_ENCODED_SIGNED_SIGNATURE] = { 0 };
  size_t encodedSignedSignatureLength = 0;

  // Get the signature. It will be signed later with the decoded device key.
  // To change the sas token duration, see IOT_CONFIG_SAS_TOKEN_EXPIRY_MINUTES in iot_configs.h
  sasTokenDuration = getSASTokenExpirationTime(IOT_CONFIG_SAS_TOKEN_EXPIRY_MINUTES);
  result = az_iot_hub_client_sas_get_signature(
    &azIoTHubClient, sasTokenDuration, signatureAzSpan, &signatureAzSpan);
  EXIT_LOOP(az_result_failed(result), "Could not get the signature for SAS Token. Return code: " + result);

  // Sign and encode the signature (b64 encoded, HMAC-SHA256 signing).
  // Uses the decoded device key.
  generateSASBase64EncodedSignedSignature(
    az_span_ptr(signatureAzSpan), az_span_size(signatureAzSpan),
    encodedSignedSignature, sizeof(encodedSignedSignature), &encodedSignedSignatureLength);

  // Get the MQTT password (SAS Token) from the base64 encoded, HMAC signed bytes.
  az_span encodedSignedSignatureAzSpan = az_span_create(encodedSignedSignature,
                                                        encodedSignedSignatureLength);
  result = az_iot_hub_client_sas_get_password(
    &azIoTHubClient, sasTokenDuration, encodedSignedSignatureAzSpan, AZ_SPAN_EMPTY,
    mqttPassword, sizeof(mqttPassword), NULL);
  EXIT_LOOP(az_result_failed(result), "Could not get the MQTT password. Return code: " + result);
}

// //
//  * generateSASBase64EncodedSignedSignature:
//  * Sign and encode a signature. It is signed using the provided device key.
//  * The process is:
//  *    1. Decode the encoded device key.
//  *    2. Sign the signature with the decoded device key.
//  *    3. Encode the signed signature.
//  *//
static void generateSASBase64EncodedSignedSignature(
  uint8_t const* sasSignature, size_t const sasSignatureSize,
  uint8_t* encodedSignedSignature, size_t encodedSignedSignatureSize,
  size_t* encodedSignedSignatureLength) {
  int result;
  unsigned char sasDecodedKey[BUFFER_LENGTH_SAS] = { 0 };
  az_span sasDecodedKeySpan = AZ_SPAN_FROM_BUFFER(sasDecodedKey);
  int32_t sasDecodedKeyLength = 0;
  uint8_t sasHMAC256SignedSignature[BUFFER_LENGTH_SAS] = { 0 };

  // Decode the SAS base64 encoded device key to use for HMAC signing.
  az_span configDeviceKeySpan = az_span_create((uint8_t*)IOT_CONFIG_DEVICE_KEY, sizeof(IOT_CONFIG_DEVICE_KEY) - 1);
  result = az_base64_decode(sasDecodedKeySpan, configDeviceKeySpan, &sasDecodedKeyLength);
  EXIT_LOOP(result != AZ_OK, "az_base64_decode failed. Return code: " + result);

  // HMAC-SHA256 sign the signature with the decoded device key.
  result = ECCX08.begin();
  EXIT_LOOP(!result, "Failed to communicate with ATECC608.");

  result = ECCX08.nonce(sasDecodedKey);
  EXIT_LOOP(!result, "Failed to do nonce.");

  result = ECCX08.beginHMAC(0xFFFF);
  EXIT_LOOP(!result, "Failed to start HMAC operation.");

  result = ECCX08.updateHMAC(sasSignature, sasSignatureSize);
  EXIT_LOOP(!result, "Failed to update HMAC with signature.");

  result = ECCX08.endHMAC(sasHMAC256SignedSignature);
  EXIT_LOOP(!result, "Failed to end HMAC operation.");

  // Base64 encode the result of the HMAC signing.
  az_span signedSignatureSpan = az_span_create(sasHMAC256SignedSignature, sizeof(sasHMAC256SignedSignature));
  az_span encodedSignedSignatureSpan = az_span_create(encodedSignedSignature, encodedSignedSignatureSize);
  result = az_base64_encode(encodedSignedSignatureSpan, signedSignatureSpan, (int32_t*)encodedSignedSignatureLength);
  EXIT_LOOP(result != AZ_OK, "az_base64_encode failed. Return code: " + result);
}

/*
 * getSASTokenExpirationTime:
 * Calculate expiration time from current time and duration value.
 */
static uint64_t getSASTokenExpirationTime(uint32_t minutes) {
  unsigned long now = getTime();                              // GMT
  unsigned long expiryTime = now + (SECS_PER_MIN * minutes);  // For SAS Token
  unsigned long localNow = now + GMT_OFFSET_SECS;
  unsigned long localExpiryTime = expiryTime + GMT_OFFSET_SECS;

  Logger.Info("UTC Current time: " + getFormattedDateTime(now) + " (epoch: " + now + " secs)");
  Logger.Info("UTC Expiry time: " + getFormattedDateTime(expiryTime) + " (epoch: " + expiryTime + " secs)");
  Logger.Info("Local Current time: " + getFormattedDateTime(localNow));
  Logger.Info("Local Expiry time: " + getFormattedDateTime(localExpiryTime));

  return (uint64_t)expiryTime;
}

/**********************************/
/*    Time and Error functions    */
/**********************************/

/*
 * getTime:
 * WiFi client returns the seconds corresponding to GMT epoch time.
 * This function used as a callback by the SSL library to validate the server certificate
 * and in SAS token generation.
 */
static unsigned long getTime() {
  return WiFi.getTime();
}

/*
 * getFormattedDateTime:
 * Custom formatting for epoch seconds. Used in logging.
 */
static String getFormattedDateTime(unsigned long epochTimeInSeconds) {
  char dateTimeString[BUFFER_LENGTH_DATETIME_STRING];

  time_t epochTimeInSecondsAsTimeT = (time_t)epochTimeInSeconds;
  struct tm* timeInfo = localtime(&epochTimeInSecondsAsTimeT);

  strftime(dateTimeString, 20, "%F %T", timeInfo);

  return String(dateTimeString);
}

/*
 * mqttErrorCodeName:
 * Legibly prints AruinoMqttClient library error enum values. 
 */
static String mqttErrorCodeName(int errorCode) {
  String errorMessage;
  switch (errorCode) {
    case MQTT_CONNECTION_REFUSED:
      errorMessage = "MQTT_CONNECTION_REFUSED";
      break;
    case MQTT_CONNECTION_TIMEOUT:
      errorMessage = "MQTT_CONNECTION_TIMEOUT";
      break;
    case MQTT_SUCCESS:
      errorMessage = "MQTT_SUCCESS";
      break;
    case MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
      errorMessage = "MQTT_UNACCEPTABLE_PROTOCOL_VERSION";
      break;
    case MQTT_IDENTIFIER_REJECTED:
      errorMessage = "MQTT_IDENTIFIER_REJECTED";
      break;
    case MQTT_SERVER_UNAVAILABLE:
      errorMessage = "MQTT_SERVER_UNAVAILABLE";
      break;
    case MQTT_BAD_USER_NAME_OR_PASSWORD:
      errorMessage = "MQTT_BAD_USER_NAME_OR_PASSWORD";
      break;
    case MQTT_NOT_AUTHORIZED:
      errorMessage = "MQTT_NOT_AUTHORIZED";
      break;
    default:
      errorMessage = "Unknown";
      break;
  }

  return errorMessage;
}
