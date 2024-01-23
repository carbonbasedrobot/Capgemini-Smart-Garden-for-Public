// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

/*--- Libraries ---*/
//include this to work with analog pins
#include <SPI.h>
#include <Arduino.h>

//included to add json functionalitypj
//#include <Arduino_JSON.h>

//tank level sensor library
//#include <Ultrasonic.h>
#include <HCSR04.h>


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

//Attempting to track tank levels through extrapolating pump activity
  //NOTE: This code relies on the user refilling the tank completely, then restarting the system
// double flowrate = 10; //in in^3/s
// double maxTankVolume = 380; //pi*5.5^2 * 4 (volume calculation)
// double extrapTankLevel = 100;
// double tankThreshold = 10; //maximum percentage of tank level


void updateTankLevel(int wateringTime){
  // extrapTankLevel = extrapTankLevel - flowrate * wateringTime / maxTankVolume; //change the tank level according to the percentage of the tank assumed to be watered
  //Serial.println("Pod turned on, assumed new tank volume" + String(extrapTankLevel)); 
}


//below is some code meant for using the ultrasonic sensor to read the tank level, running into random output values not seen when library is used in standalone tests
//define variables used for reading and reporting about tank levels
// int tankLevel;
// int tankThreshold = 10;          //don't water if level is lower than this (in percentage)
// int tankSensorHeight = 11;

double tankLevel;
double tankThreshold = 10;          //don't water if level is lower than this (in percentage)
double tankSensorHeight = 11.64; //how far the tank sensor is from the bottom in CM

UltraSonicDistanceSensor distanceSensor(3, 2);  // Initialize sensor that uses digital pins 13 and 12.
double tempWaterLevel;
double mappedValue;
double sensorReading;

double readTankHeight() { 
  delay(50);
  tankSensorHeight = 11.64;
  sensorReading = distanceSensor.measureDistanceCm();
  if(sensorReading > tankSensorHeight){
    sensorReading = tankSensorHeight; //deal with overshooting readings
  }
    if(sensorReading < 3){
    sensorReading = 3; //deal with undershooting readings
  }
 // sensorReading = sensorReading - 3; //shift range of values from 11.64 to 3 into 8.64 when empty and 0 when full

 // sensorReading = (tankSensorHeight-3-sensorReading)/(tankSensorHeight-3)*100; //convert 8.64 when empty to 0% and and 0 when empty to 100%

  return sensorReading;
}

bool noZonesActive = true;  // global variable used in  and checkAndWater used to check that all zones aren't running to avoid running more than one zone at once

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
    _moistureValue;                          //default value gets changed after analog value is read
    _startedWateringTime;                    //comparison timestamp, set to millis()/1000 when the pump is turned on
    _isWatering = false;                     // value is change to true when solenoid/pump are onrea
    _lowCalValue = lowCalValue;
    _highCalValue = highCalValue;
  }


  // returns a moisture value from 0-100 that is calibrated according to the zones moisture sensor
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
      moistureReading = analogRead(A5);
    } else {
      moistureReading = analogRead(A6);
    }
    //limit misbehavior from the mapping function by limiting signal values to range
    if(moistureReading < _lowCalValue){
      moistureReading = _lowCalValue;
    }
    if(moistureReading > _highCalValue){
      moistureReading = _highCalValue;
    }
    //_moistureValue = map(moistureReading, _lowCalValue, _highCalValue, 100, 0);
    _moistureValue = moistureReading;
            // Moisture Sensor Testing 1/17
        // Pins: Wet   Dry
        // A0:   120   694
        // A1:   not working
        // A2:   400   880
        // A3:   not working
        // A4:   not working
        // A5:   not working
        // A6:   450   1023
        // Solution: improve/redo the wiring for the moisture sensors
  }

  void setTankLevel() {
    _tankLevel = readTankHeight();
    //_tankLevel = extrapTankLevel;
  }

  double getTankLevel() {
    return readTankHeight();
  }

  void checkAndWater() {

    _tankLevel = readTankHeight();
   // _tankLevel = extrapTankLevel;
    int pumpPin = 5;
    int currentMillis = millis() / 1000;  //current time variable for comparison in seconds
    readMoisturePin();                    //update the zones current moisture value by reading sensor
    
    if (currentMillis - _lastWateringTime >= _minWateringGap && _zoneInstalled) { // if it has been long enough since it was last watered and the zone has been installed by the user
      if (_moistureValue < _moistureThreshold || currentMillis - _lastWateringTime >= _maxWateringGap) {  //and the moisture value is below the threshold or its been too long enough
        if (_isWatering == false && _tankLevel > tankThreshold) {                                         //tank isn't too low and its not already watering
          _isWatering = true;
          digitalWrite(_relayPin, HIGH);
          delay(500);  //to avoid malfunction
          digitalWrite(pumpPin, HIGH);
          _startedWateringTime = millis() / 1000;
          Serial.println("zone " + String(_zoneNumber) + " was turned on");
          //update tank level variable according to extrapolation
          //updateTankLevel(_wateringTime);
          //_tankLevel = extrapTankLevel;
          noZonesActive = false;
        }
      }
    }
    //need to turn off if level is too low
    if (_tankLevel < tankThreshold) {
      delay(500);
      digitalWrite(pumpPin, LOW);
      delay(500);  //to avoid a malfunction
      digitalWrite(_relayPin, LOW);
      _lastWateringTime = millis() / 1000;  //record the most recent time the plant was watered
      _isWatering = false;
      noZonesActive = true;  //only one zone will work at a time, so set it to true after turning off the watering
      Serial.println("Pump was turned off because the tank was too low");
    }
    //turns off pump/valve if enough water has been sent
    if (_isWatering && millis() / 1000 - _startedWateringTime > _wateringTime) {
      digitalWrite(pumpPin, LOW);
      delay(500);  //to avoid a malfunction
      digitalWrite(_relayPin, LOW);
      _lastWateringTime = millis() / 1000;  //record the most recent time the plant was watered
      _isWatering = false;
      noZonesActive = true;  //only one zone will work at a time, so set it to true after turning off the watering
      Serial.println("Finished Watering Zone " + String(_zoneNumber));
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

  void setZoneInstalled(bool zoneInstalled) {
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
  String getZoneInfo() {  //edited to only give necessary info to azure api
    String zoneInfo;
    delay(10);
    zoneInfo = "Zone: " + String(_zoneNumber) + "; ";
    zoneInfo += "Moisture Value: " + String(_moistureValue);
    zoneInfo += "; Is watering: " + String(_isWatering);
    zoneInfo += "; Last Watering Time: " + String(_lastWateringTime) + "; ";
    zoneInfo += "; Relay Pin: " + String(_relayPin) + "; ";
    zoneInfo += "; Zone is Installed: " + String(_zoneInstalled);
    zoneInfo += "; Moisture Threshold: " + String(_moistureThreshold) + "; ";
    zoneInfo += "; Watering Time: " + String(_wateringTime) + "; ";
    zoneInfo += "; Min Watering Gap: " + String(_minWateringGap) + "; ";
    zoneInfo += "; Max Watering Gap: " + String(_maxWateringGap) + "; ";
    zoneInfo += "; Tank Level: " + String(readTankHeight());
    //return zoneInfo + "; Current millis in seconds: " + String(millis() / 1000);
    return zoneInfo;
  }

  String getJSONZoneInfo() {

    
    String zoneData = "";
    zoneData += String(_zoneNumber) + ',';
    readMoisturePin();
    zoneData += String(_moistureValue) + ',';
    zoneData += String(_isWatering) + ',';
    zoneData += String(_lastWateringTime) + ',';
    setTankLevel();
    zoneData += String(_tankLevel);
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
  double _tankLevel;
};

//default values
bool defInstallStatus = 0;  //when the arduino starts up, don't want to start watering potentiall empty pods
int defMoistureThreshold = 35;
int defWateringTime = 1;
int defMinWateringGap = 5;
int defMaxWateringGap = 10;
int defLowCalValue = 250;
int defHighCalValue = 775;


//initialize wateringzones with
WateringZone zoneOne = WateringZone(1, 12, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneTwo = WateringZone(2, 11, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneThree = WateringZone(3, 10, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneFour = WateringZone(4, 9, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneFive = WateringZone(5, 8, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneSix = WateringZone(6, 7, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);
WateringZone zoneSeven = WateringZone(7, 6, defInstallStatus, defMoistureThreshold, defWateringTime, defMinWateringGap, defMaxWateringGap, defLowCalValue, defHighCalValue);



WateringZone zoneArray[7] = { zoneOne, zoneTwo, zoneThree, zoneFour, zoneFive, zoneSix, zoneSeven };

void setup() {  //code that only runs once

  Serial.begin(SERIAL_LOGGER_BAUD_RATE);

  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    initializeAzureIoTHubClient();
    initializeMQTTClient();
    connectMQTTClientToAzureIoTHub();
  }

  //digitalWrite(LED_PIN, LOW);
  telemetryNextSendTimeMs = 0;

  //initializing the digital pins used to control the relay
  for (int i = 4; i < 12; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
    Serial.println("pin initialized: " + String(i));
  }
  //set up pin 13 to power sensors
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}
/*
 * loop:
 * Check for connection and reconnect if necessary.
 * Send Telemetry and receive messages.
 */
 int attemptCount = 0;
void loop() {
  // code that runs repeatedly  disconnected from power
  tempWaterLevel = readTankHeight(); //variable used to detect changes in tanklevel
  Serial.println("Temp Water Level: " + String(tempWaterLevel));
  if (!Serial) {  //reconnect to serial if disconnected
    Serial.begin(SERIAL_LOGGER_BAUD_RATE);
  }

  if (WiFi.status() != WL_CONNECTED) {  //tries to connect to the cloud if not connected
    if(attemptCount > 0){
       for(int i = 0; i < 7; i++){
       Serial.println(zoneArray[i].getZoneInfo());
       }
    }
    connectToWiFi();
    attemptCount += 1;
    Serial.println("Attempt Count: " + String(attemptCount));
    if (WiFi.status() == WL_CONNECTED) {  // IF CONNECTION SUCCEEDS, BUT IT WAS DISCONNECTED AT SOME POINT, THIS INITIALIZES AZURE THINGS
      initializeAzureIoTHubClient();
      initializeMQTTClient();
      connectMQTTClientToAzureIoTHub();
    }
  } else {          //if its already connected
      if (millis() > telemetryNextSendTimeMs || abs(zoneArray[0].getTankLevel() - tempWaterLevel) > 5)  //if its been long enough since the last message was sent or the tank level has changed by 5%
      //  if(millis() > telemetryNextSendTimeMs)
      {
        if (abs(zoneArray[0].getTankLevel() - tempWaterLevel) > 5) {
          Serial.println("temp level: " + String(tempWaterLevel)+ "; current tank level: " + String(zoneArray[0].getTankLevel())) ;
        }
        //Serial.println("time: " + String(millis()/1000) + "Tank level: " + String(tankLevel));
        // Check for MQTT Client connection to Azure IoT hub. Reconnect if needed.
        if (!mqttClient.connected()) {
          connectMQTTClientToAzureIoTHub();
        }
        sendTelemetry();  //triggers telemetryPayload to be filled with zone info, then sends it
        telemetryNextSendTimeMs = millis() + IOT_CONFIG_TELEMETRY_FREQUENCY_MS;
      }

    // MQTT loop must be called to process Telemetry and Cloud-to-Device (C2D) messages.
    mqttClient.poll();
  }
  
  for (int i = 0; i < 7; i++) { //update sensor values in case a zone is already active
    zoneArray[i].readMoisturePin();
    zoneArray[i].setTankLevel();
  }

  for(int i = 0; i < 7; i++){ //check each zone
    if(zoneArray[i].getIsWatering()){ //check if its been watering long enough
      zoneArray[i].checkAndWater();
    }

    if(noZonesActive){ //check if the zone should be watered
      zoneArray[i].checkAndWater();
    }
  }
  // //this loop checks if any zones are active. 
  // noZonesActive = true;
  // for(int i = 0; i < 7; i++){ /determine whether or not a zone is currently being watered
  //   if(zoneArray[i].getIsWatering()){
  //     noZonesActive = false;
  //     zoneArray[i].checkAndWater();
  //     break; //loop, run check and water on the still active zone, no need to check other zones
  //   }
  // }

  //   for(int i = 0; i < 7; i++){ //run the check and water function on all of the zones<7
  //     if(noZonesActive){
  //       zoneArray[i].checkAndWater();
  //       if(zoneArray[i].getIsWatering()){
  //         noZonesActive = false;
  //       }
  //   }
 // }
  // // if (tankLevel < tankThreshold) {  //read all moisture pins if tank is too low
  // //   for (int i = 0; i < 7; i++) {
  // //     zoneArray[i].readMoisturePin();
  // //   }
  // } else {
  //   //checks if a zone is already watering or not
  //   for (int i = 0; i < 7; i++) {  //if any of the zones are already running
  //     if (zoneArray[i].getIsWatering()) {
  //       noZonesActive = false;
  //       zoneArray[i].checkAndWater();  //need to check if its been long enough if the pump is currently running
  //       i = 9;                         //break because we already know there is a petal running
  //     }
  //   }
  //   if (noZonesActive) {  // only runs if no zones are currently being watered
  //     for (int j = 0; j < 7; j++) {
  //       zoneArray[j].checkAndWater();        //check to see if its time to water
  //       if (zoneArray[j].getIsWatering()) {  //if this causes it to start watering, no need to check other zones
  //         j = 9;                             //break because we already checked the zone and its now watering
  //       }
  //     }
  //  }

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
  } else {
    Serial.println("Connection Attempt Failed");
  }
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
  while (mqttClient.available())  //fills recievedMessage, one char at a time
  {
    char recievedChar = (char)mqttClient.read();
    recievedMessage += recievedChar;
    // Serial.print((char)mqttClient.read());
  }
  Serial.println(recievedMessage);

  JSONVar zonesData = JSON.parse(recievedMessage);  //parses the json messge

  for (int i = 0; i < 7; i++) {
    JSONVar zoneData = zonesData[i];
    //extract zone specific data
    zoneArray[i].setZoneInstalled(zoneData["is_pod_active"]);
    zoneArray[i].setWateringTime(zoneData["watering_duration"]);
    zoneArray[i].setMinWateringGap(zoneData["min_water_gap"]);
    zoneArray[i].setMaxWateringGap(zoneData["max_water_gap"]);
    zoneArray[i].setMoistureThreshold(zoneData["moisture_threshold"]);
  }
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

  //fill json array with json lists of zone information
  String zoneInfo = "";
  for (int i = 0; i < 7; i++) {
    //Serial.println(zoneInfo);
    zoneInfo += zoneArray[i].getJSONZoneInfo() + ";";
  }
  //convert json array into usable format
  Serial.println("Telemetery being sent: " + zoneInfo);
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
    size_t* encodedSignedSignatureLength) 
{
  int result;
  unsigned char sasDecodedKey[BUFFER_LENGTH_SAS] = {0};
  az_span sasDecodedKeySpan = AZ_SPAN_FROM_BUFFER(sasDecodedKey);
  int32_t sasDecodedKeyLength = 0;
  uint8_t sasHMAC256SignedSignature[BUFFER_LENGTH_SAS] = {0};

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
  result = az_base64_encode(encodedSignedSignatureSpan, signedSignatureSpan, (int32_t*) encodedSignedSignatureLength);
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
