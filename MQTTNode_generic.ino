

// TODO come up with standard Sensor names
// TODO setup standard actuator/output structure so nodes can also do something
// Note: Suggest extra 50k in series with A0 to measure 3.3v from ATX power supply
// this might reduce resolution though

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Rev 7 MQTT sensor node
//  Features:
//  OTA updates
//  -Standard MQTT information prefixed by Node Name
//  -Also sent via MQTT node information including Sketch name, node name,IP address, Wifi Station connected to,
//    Location of Sensor - to allow nodes to be identified via MQTT and IP address determined for OTA update
//  -Sensor description also sent via MQTT for setting up what ever system will read MQTT data
//    i.e. OpenHab so if its a temperature sensor you can setup it without needing to open sketch to see what type of data it is
//
//  Includes new function Serial output via mqtt topic is mqttNode + "/info/Serial"
//  required
//    #ifdef SerialViaMqqt
//      mqttSerialData="message to go to mqtt";
//      mqttNodeSerial();
//    #endif
//
// Added subscribe topic in defines - Current implementation assumes only one topic being subscribed in one application
// Next version will include multiple topics
//

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Defines for determining what will be compiled
//  Allows for customisation of SensorNodes and some testing
// Specific Sensor read code needs to be added to function ReadSensors()
//
//  Not sure if I've done this in the most efficient way but its a start



#define Sensor1_exists
//#define Sensor2_exists  //uncomment if using Sensor2
//#define Sensor3_exists  //uncomment if using Sensor3
//#define Sensor4_exists  //uncomment if using Sensor4
//#define Sensor5_exists  //uncomment if using Sensor5
//#define WIFIOFF   // for testing without wifi
//#define mqttOFF   // for testing without mqtt
//#define SerialViaMqqt //Send serial data to mqqt topic

////////////////////////////////////////////////////////////////////////////////////////////////////
// Modify these values for your environment



const char* otaPassword = "abc123";  // OTA update password
char* mqttServer = "192.168.1.1";   //  Your MQTT server IP address
char* mqttUser = "";  //mqtt username, set to "" for no user
char* mqttPassword = "";  //mqtt password, set to "" for no password


const String mqttNode = "MQTTNode99"; // Your unique hostname for this device
const String mqttLocation = "Shed"; // Location for this device
const String mqttSketch = "abc123"; // Sketch Name for this Node
const String mqttSensor1type = "Voltage sensor"; // Sensor type
const String mqttSensor2type = ""; // Sensor type
const String mqttSensor3type = ""; // Sensor type
const String mqttSensor4type = ""; // Sensor type
const String mqttSensor5type = ""; // Sensor type
////////////////////////////////////////////////////////////////////////////////////////////////////

// Standard MQTT information sent at low frequency
const String mqttIPTopic = mqttNode + "/info/IPAddress";
const String mqttWiFiAPTopic = mqttNode + "/info/WiFiAP";
const String mqttSketchTopic = mqttNode + "/info/Sketch";
const String mqttLocationTopic = mqttNode + "/info/Location";
const String mqttSerialPrintTopic = mqttNode + "/info/Serial";
const String mqttSubTopic = mqttNode + "/actuator/1";

String mqttSerialData;


// Mqtt Sensor topics
const String mqttSensor1Topic = mqttNode + "/sensor/Sensor1";
const String mqttSensor2Topic = mqttNode + "/sensor/Sensor2";
const String mqttSensor3Topic = mqttNode + "/sensor/Sensor3";
const String mqttSensor4Topic = mqttNode + "/sensor/Sensor4";
const String mqttSensor5Topic = mqttNode + "/sensor/Sensor5";
//  Mqtt Sensor description topic
const String mqttSensor1_info_Topic = mqttNode + "/sensor_type/Sensor1";
const String mqttSensor2_info_Topic = mqttNode + "/sensor_type/Sensor2";
const String mqttSensor3_info_Topic = mqttNode + "/sensor_type/Sensor3";
const String mqttSensor4_info_Topic = mqttNode + "/sensor_type/Sensor4";
const String mqttSensor5_info_Topic = mqttNode + "/sensor_type/Sensor5";

//  Node status topics
// And a sensor for WiFi signal strength
const String mqttWiFiSignalTopic = mqttNode + "/status/WiFiSignal";
// And a sensor for device uptime
const String mqttUptimeTopic = mqttNode + "/status/uptime";


// Set the signal strength and uptime reporting interval in milliseconds
const unsigned long reportInterval = 30000;
const unsigned long NodeInfoInterval = 300000;
unsigned long reportTimer = millis();
unsigned long NodeInfoTimer = millis();

// Set LED "twinkle" time for maximum daylight visibility
const unsigned long twinkleInterval = 50;
unsigned long twinkleTimer = millis();

// Define Sensor Variables
float Sensor1;
float Sensor2;
float Sensor3;
float Sensor4;
float Sensor5;


//Ported to ESP32
#ifdef ESP32
#include <FS.h>
#include "SPIFFS.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_BUILTIN       2
#define LED_ON            HIGH
#define LED_OFF           LOW

#else
#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>

#define ESP_getChipId()   (ESP.getChipId())

#define LED_ON      LOW
#define LED_OFF     HIGH
#endif
#define PIN_LED       LED_BUILTIN
#include <ArduinoOTA.h>
#include <ESP_WiFiManager.h>
//#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

char configFileName[] = "/config.json";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// SSID and PW for Config Portal
String AP_SSID;
String AP_PASS;

//define your default values here, if there are different values in configFileName (config.json), they are overwritten.
#define BLYNK_SERVER_LEN                64
#define BLYNK_TOKEN_LEN                 32
#define BLYNK_SERVER_PORT_LEN           6

#define MQTT_SERVER_MAX_LEN             40
#define MQTT_SERVER_PORT_LEN            6

char blynk_server [BLYNK_SERVER_LEN]        = "account.duckdns.org";
char blynk_port   [BLYNK_SERVER_PORT_LEN]   = "8080";
char blynk_token  [BLYNK_TOKEN_LEN]         = "YOUR_BLYNK_TOKEN";

char mqtt_server  [MQTT_SERVER_MAX_LEN];
char mqtt_port    [MQTT_SERVER_PORT_LEN]    = "8080";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback(void)
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}




//define your default values here, if there are different values in configFileName (config.json), they are overwritten.
#define BLYNK_SERVER_LEN                64
#define BLYNK_TOKEN_LEN                 32
#define BLYNK_SERVER_PORT_LEN           6

#define MQTT_SERVER_MAX_LEN             40
#define MQTT_SERVER_PORT_LEN            6




bool loadSPIFFSConfigFile(void)
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println("Mounted file system");

    if (SPIFFS.exists(configFileName))
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open(configFileName, "r");

      if (configFile)
      {
        Serial.print("Opened config file, size = ");
        size_t configFileSize = configFile.size();
        Serial.println(configFileSize);

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);

        configFile.readBytes(buf.get(), configFileSize);

        Serial.print("\nJSON parseObject() result : ");

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);

        if ( deserializeError )
        {
          Serial.println("failed");
          return false;
        }
        else
        {
          Serial.println("OK");

          if (json["blynk_server"])
            strncpy(blynk_server, json["blynk_server"], sizeof(blynk_server));

          if (json["blynk_port"])
            strncpy(blynk_port, json["blynk_port"], sizeof(blynk_port));

          if (json["blynk_token"])
            strncpy(blynk_token,  json["blynk_token"], sizeof(blynk_token));

          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));
        }

        //serializeJson(json, Serial);
        serializeJsonPretty(json, Serial);
#else
        DynamicJsonBuffer jsonBuffer;
        // Parse JSON string
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        // Test if parsing succeeds.

        if (json.success())
        {
          Serial.println("OK");

          if (json["blynk_server"])
            strncpy(blynk_server, json["blynk_server"], sizeof(blynk_server));

          if (json["blynk_port"])
            strncpy(blynk_port, json["blynk_port"], sizeof(blynk_port));

          if (json["blynk_token"])
            strncpy(blynk_token,  json["blynk_token"], sizeof(blynk_token));

          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));
        }
        else
        {
          Serial.println("failed");
          return false;
        }
        //json.printTo(Serial);
        json.prettyPrintTo(Serial);
#endif

        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    return false;
  }
  return true;
}

bool saveSPIFFSConfigFile(void)
{
  Serial.println("Saving config");

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif

  json["blynk_server"] = blynk_server;
  json["blynk_port"]   = blynk_port;
  json["blynk_token"]  = blynk_token;

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"]   = mqtt_port;

  File configFile = SPIFFS.open(configFileName, "w");

  if (!configFile)
  {
    Serial.println("Failed to open config file for writing");
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  //serializeJson(json, Serial);
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, configFile);
#else
  //json.printTo(Serial);
  json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(configFile);
#endif

  configFile.close();
  //end save
}

void heartBeatPrint(void)
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print("H");        // H means connected to WiFi
  else
    Serial.print("F");        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(" ");
  }
}

void toggleLED()
{
  //toggle state
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong LEDstatus_timeout    = 0;
  static ulong currentMillis;

#define HEARTBEAT_INTERVAL    10000L
#define LED_INTERVAL          2000L

  currentMillis = millis();

  if ((currentMillis > LEDstatus_timeout) || (LEDstatus_timeout == 0))
  {
    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = currentMillis + LED_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((currentMillis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = currentMillis + HEARTBEAT_INTERVAL;
  }
}


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);



////////////////////////////////////////////////////////////////////////////////////////////////////
// System setup
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);

  Serial.println("\nHardware initialized, starting program load");
  Serial.println("Macka Standard MQTT node7");
  Serial.println("==============================");
  Serial.println(" ");

  //read configuration from FS json
  Serial.println("mounting FS...");
  loadSPIFFSConfigFile();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  ESP_WMParameter custom_blynk_server("blynk_server", "blynk_server", blynk_server, BLYNK_SERVER_LEN + 1);
  ESP_WMParameter custom_blynk_port  ("blynk_port",   "blynk_port",   blynk_port,   BLYNK_SERVER_PORT_LEN + 1);
  ESP_WMParameter custom_blynk_token ("blynk_token",  "blynk_token",  blynk_token,  BLYNK_TOKEN_LEN + 1 );

  ESP_WMParameter custom_mqtt_server("mqtt_server", "mqtt_server", mqtt_server, MQTT_SERVER_MAX_LEN + 1);
  ESP_WMParameter custom_mqtt_port  ("mqtt_port",   "mqtt_port",   mqtt_port,   MQTT_SERVER_PORT_LEN + 1);

  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("AutoConnect-FSParams");

  //set config save notify callback
  ESP_wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  ESP_wifiManager.addParameter(&custom_blynk_server);
  ESP_wifiManager.addParameter(&custom_blynk_port);
  ESP_wifiManager.addParameter(&custom_blynk_token);

  ESP_wifiManager.addParameter(&custom_mqtt_server);
  ESP_wifiManager.addParameter(&custom_mqtt_port);

  //reset settings - for testing
  //ESP_wifiManager.resetSettings();

  ESP_wifiManager.setDebugOutput(true);

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //ESP_wifiManager.setMinimumSignalQuality();

  //set custom ip for portal
  ESP_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 100, 1), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));

  ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5+
  ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192, 168, 2, 114), IPAddress(192, 168, 2, 1), IPAddress(255, 255, 255, 0),
                                       IPAddress(192, 168, 2, 1), IPAddress(8, 8, 8, 8));

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("\nStored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 120s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
  }

  String chipID = String(ESP_getChipId(), HEX);
  chipID.toUpperCase();

  // SSID and PW for Config Portal
  AP_SSID = "ESP_" + chipID + "_AutoConnectAP";
  AP_PASS = "MyESP_" + chipID;

  // Get Router SSID and PASS from EEPROM, then open Config portal AP named "ESP_XXXXXX_AutoConnectAP" and PW "MyESP_XXXXXX"
  // 1) If got stored Credentials, Config portal timeout is 60s
  // 2) If no stored Credentials, stay in Config portal until get WiFi Credentials
  if (!ESP_wifiManager.autoConnect(AP_SSID.c_str(), AP_PASS.c_str()))
  {
    Serial.println("failed to connect and hit timeout");

    //reset and try again, or maybe put it to deep sleep
#ifdef ESP8266
    ESP.reset();
#else   //ESP32
    ESP.restart();
#endif
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected");

  //read updated parameters
  //  strncpy(blynk_server, custom_blynk_server.getValue(), sizeof(blynk_server));
  //  strncpy(blynk_port,   custom_blynk_port.getValue(),   sizeof(blynk_port));
  //  strncpy(blynk_token,  custom_blynk_token.getValue(),  sizeof(blynk_token));

  strncpy(mqttServer, custom_mqtt_server.getValue(), sizeof(mqtt_server));
  strncpy(mqtt_port, custom_mqtt_port.getValue(),     sizeof(mqtt_port));

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveSPIFFSConfigFile();
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

#ifndef mqttOFF
  // Create server and assign callbacks for MQTT
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(mqtt_callback);
#endif


  // Start up OTA
  if (otaPassword[0]) {
    setupOTA();
  }

  // Display What Sensors are defined on Serial console
#ifdef Sensor1_exists
  Serial.println("Using Sensor1 of type: " + mqttSensor1type);
#endif
#ifdef Sensor2_exists
  Serial.println("Using Sensor2 of type: " + mqttSensor2type);
#endif
#ifdef Sensor3_exists
  Serial.println("Using Sensor3 of type: " + mqttSensor3type);
#endif
#ifdef Sensor4_exists
  Serial.println("Using Sensor4 of type: " + mqttSensor4type);
#endif
#ifdef Sensor5_exists
  Serial.println("Using Sensor5 of type: " + mqttSensor5type);
#endif
  Serial.println(" ");
  Serial.println("Initialization complete\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // check WiFi connection
  //Do not compile if Wifi  not being used
#ifndef WIFIOFF
  if (WiFi.status() != WL_CONNECTED) {
  }
  //Do not compile if Wifi or mqtt not being used
#ifndef mqttOFF
  // check MQTT connection
  if (!mqttClient.connected()) {
    mqttConnect();
  }

  // MQTT client loop
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
#endif
#endif

  // LED twinkle
  /*  if (((millis() - twinkleTimer) >= twinkleInterval)) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      twinkleTimer = millis();
    }
  */
  // Report signal strength and uptime
  if (((millis() - reportTimer) >= reportInterval)) {
    ReadSensors(); //Read sensors

    //Do not compile if Wifi or mqtt not being used
#ifndef WIFIOFF
#ifndef mqttOFF
    if (mqttClient.connected()) {
      mqttNodeStatus(); // Send Node status data to MQTT Server
      mqttNodeSensorData(); // Send Sensor data to MQTT Server
    }
#endif
#endif


    reportTimer = millis();
  }
  //Do not compile if Wifi or mqtt not being used
#ifndef WIFIOFF
#ifndef mqttOFF
  // Report Node Information
  if (mqttClient.connected() && ((millis() - NodeInfoTimer) >= NodeInfoInterval)) {
    mqttNodeInfo();

    NodeInfoTimer = millis();
  }

  // OTA loop
  if (otaPassword[0]) {
    ArduinoOTA.handle();
  }
#endif
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions

////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle incoming commands from MQTT
void mqtt_callback(char* topic, byte* payload, unsigned int payloadLength) {
  //Current implementation assumes only one topic being subscribed at the moment
  // Next version will include multiple topics

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#ifdef SerialViaMqqt
  char* payload1;
  mqttSerialData = "Message arrived [" + topic + "] " + payload ;
  for (int i = 0; i < payloadLength; i++) {
    payload1 = payload1(char)payload[i];
  }
  mqttSerialData = mqttSerialData + payload1;
  mqttNodeSerial();
#endif

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(D1, HIGH);   // Turn the ATX relay on
  } else {
    digitalWrite(D1, LOW);  // Turn the ATX relay off
  }

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MQTT connection and subscriptions
void mqttConnect() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Attempting MQTT connection to broker: " + String(mqttServer));
  // Attempt to connect to broker, setting last will and testament
  if (mqttClient.connect(mqttNode.c_str(), mqttUser, mqttPassword)) {

    // when connected, record Node Info, signal strength and reset reporting timer
    // publish MQTT Node Info and Node Status
    mqttNodeInfo();
    mqttNodeStatus();
    reportTimer = millis();
    NodeInfoTimer = millis();

    Serial.println("MQTT connected");
#ifdef SerialViaMqqt
    mqttSerialData = "MQTT connected";
    mqttNodeSerial();
#endif
    digitalWrite(LED_BUILTIN, LOW);
    mqttClient.subscribe(mqttSubTopic.c_str());
  }
  else {
    Serial.println("MQTT connection failed, rc=" + String(mqttClient.state()));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// (mostly) boilerplate OTA setup from library examples
void setupOTA() {
  // Start up OTA
  // ArduinoOTA.setPort(8266); // Port defaults to 8266
  ArduinoOTA.setHostname(mqttNode.c_str());
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("ESP OTA:  update start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("ESP OTA:  update complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.println("ESP OTA:  ERROR code " + String(error));
    if (error == OTA_AUTH_ERROR) Serial.println("ESP OTA:  ERROR - Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("ESP OTA:  ERROR - Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("ESP OTA:  ERROR - Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("ESP OTA:  ERROR - Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("ESP OTA:  ERROR - End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("ESP OTA:  Over the Air firmware update ready");
}

void mqttNodeInfo() {

  Serial.println("Sending mqttNodeInfo");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.SSID());
  Serial.println(mqttSketch);
  Serial.println(mqttLocation);
  String IP = String(WiFi.localIP().toString());
  String WifiSTA = String(WiFi.SSID());


  mqttClient.publish(mqttIPTopic.c_str(), IP.c_str());
  mqttClient.publish(mqttWiFiAPTopic.c_str(), WifiSTA.c_str());
  mqttClient.publish(mqttSketchTopic.c_str(), mqttSketch.c_str());
  mqttClient.publish(mqttLocationTopic.c_str(), mqttLocation.c_str());

#ifdef Sensor1_exists
  mqttClient.publish(mqttSensor1_info_Topic.c_str(), mqttSensor1type.c_str());
#endif
#ifdef Sensor2_exists
  mqttClient.publish(mqttSensor2_info_Topic.c_str(), mqttSensor2type.c_str());
#endif
#ifdef Sensor3_exists
  mqttClient.publish(mqttSensor3_info_Topic.c_str(), mqttSensor3type.c_str());
#endif
#ifdef Sensor4_exists
  mqttClient.publish(mqttSensor4_info_Topic.c_str(), mqttSensor4type.c_str());
#endif
#ifdef Sensor5_exists
  mqttClient.publish(mqttSensor5_info_Topic.c_str(), mqttSensor5type.c_str());
#endif
}

void mqttNodeStatus() {

  Serial.println("Sending mqttNodeStatus");
  String signalStrength = String(WiFi.RSSI());
  //reportTimer = millis();
  String uptimeTimer = String(millis());
  Serial.println("MQTT WiFi signal strength: [" + mqttWiFiSignalTopic + "] : " + WiFi.RSSI());
  Serial.println("MQTT uptime : [" + mqttUptimeTopic + "] : " + uptimeTimer);
  mqttClient.publish(mqttUptimeTopic.c_str(), uptimeTimer.c_str());
  mqttClient.publish(mqttWiFiSignalTopic.c_str(), signalStrength.c_str());
}

void mqttNodeSensorData() {
  Serial.println("Sending mqttSensorData");
#ifdef SerialViaMqqt
  mqttSerialData = "Sending mqttSensorData";
  mqttNodeSerial();
#endif

#ifdef Sensor1_exists
  Serial.println("MQTT Sensor: [" + mqttSensor1Topic + "] : " + String(Sensor1).c_str());
  mqttClient.publish(mqttSensor1Topic.c_str(), String(Sensor1).c_str());
#endif
#ifdef Sensor2_exists
  Serial.println("MQTT Sensor: [" + mqttSensor2Topic + "] : " + String(Sensor2).c_str());
  mqttClient.publish(mqttSensor2Topic.c_str(), String(Sensor2).c_str());
#endif
#ifdef Sensor3_exists
  Serial.println("MQTT Sensor: [" + mqttSensor3Topic + "] : " + String(Sensor3).c_str());
  mqttClient.publish(mqttSensor3Topic.c_str(), String(Sensor3).c_str());
#endif
#ifdef Sensor4_exists
  Serial.println("MQTT Sensor: [" + mqttSensor4Topic + "] : " + String(Sensor4).c_str());
  mqttClient.publish(mqttSensor4Topic.c_str(), String(Sensor4).c_str());
#endif
#ifdef Sensor5_exists
  Serial.println("MQTT Sensor: [" + mqttSensor5Topic + "] : " + String(Sensor5).c_str());
  mqttClient.publish(mqttSensor5Topic.c_str(), String(Sensor5).c_str());
#endif

}

void ReadSensors() {
  Serial.println("Entering Sensor read section...");
#ifdef SerialViaMqqt
  mqttSerialData = "Entering Sensor read section...";
  mqttNodeSerial();
#endif

#ifdef Sensor1_exists
  // Insert Sensor 1 read code here depending on type of sensor
  ;
  Sensor1 = analogRead(A0);
  Serial.println("Sensor1: [" + mqttSensor1type + "] : " + String(Sensor1).c_str());
#endif
#ifdef Sensor2_exists
  // Insert Sensor 2 read code here depending on type of sensor
  Sensor2 = 0.0;
  Serial.println("Sensor2: [" + mqttSensor2type + "] : " + String(Sensor2).c_str());
#endif
#ifdef Sensor3_exists
  // Insert Sensor 3 read code here depending on type of sensor
  Sensor3 = 0.0;
  Serial.println("Sensor3: [" + mqttSensor3type + "] : " + String(Sensor3).c_str());
#endif
#ifdef Sensor4_exists
  // Insert Sensor 4 read code here depending on type of sensor
  Sensor4 = 0.0;
  Serial.println("Sensor4 Read: [" + mqttSensor4type + "] : " + String(Sensor4).c_str());
#endif
#ifdef Sensor5_exists
  // Insert Sensor 5 read code here depending on type of sensor
  Sensor5 = 0.0;
  Serial.println("Sensor5 Read: [" + mqttSensor5type + "] : " + String(Sensor5).c_str());
#endif

}

void mqttNodeSerial() {

  String uptimeTimer = String(millis());
  String SerialPayload = uptimeTimer + " " + mqttSerialData;

  mqttClient.publish(mqttSerialPrintTopic.c_str(), SerialPayload.c_str());

}
