Generic ESP8266 or ESP32 program using Arduino IDE
Using MQTT to send data, OTA updates and using WifiManager to configure credentials instead of hard coding them

Designed to be the template for many different sensors and actuators
Originally designed for use with OpenHab2, but there is no add-on thing for it, and can be used for any system that uses MQTT


Current code features




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
