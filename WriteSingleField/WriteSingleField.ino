/*
  WriteSingleField
  
  Description: Writes a value to a channel on ThingSpeak every 20 seconds.
  
  Hardware: ESP8266 based boards
  
  !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details. !!!
  
  Note:
  - Requires ESP8266WiFi library and ESP8622 board add-on. See https://github.com/esp8266/Arduino for details.
  - Select the target hardware from the Tools->Board menu
  - This example is written for a network using WPA encryption. For WEP or WPA, change the WiFi.begin() call accordingly.
  
  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize, and 
  analyze live data streams in the cloud. Visit https://www.thingspeak.com to sign up for a free account and create a channel.  
  
  Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
  See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.
  
  For licensing information, see the accompanying license file.
  
  Copyright 2020, The MathWorks, Inc.
*/

#include <ESP8266WiFi.h>
#include "secrets.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include <ArduinoJson.h>
  float hum = 0.0;
  float tem = 0.0;
  float value = 0.0;
  float per = 0.0;
  float alt = 0.0;
  float lum = 0.0;
  float lat = 0.0;
  float lon = 0.0; 
  float soil=0.0;
  float aud =0.0;
  
char ssid[] = SECRET_SSID;   // your network SSID (name) 
char pass[] = SECRET_PASS;   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

int number = 0;

void setup() {
  // open serial for monitoring
  Serial.begin(115200);
  while (!Serial) {
    ;// wait for serial port to connect. Needed for Leonardo native USB port only
  }
  
  WiFi.mode(WIFI_STA); 
  ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop() {

  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
   StaticJsonBuffer<1000> jsonBuffer;
    JsonObject& data = jsonBuffer.parseObject(Serial);

  if (data == JsonObject::invalid()) {
    //Serial.println("Invalid Json Object");
    jsonBuffer.clear();
    return;
  }

  //Serial.println("JSON Object Recieved");
  hum = data["humidity"];
  tem = data["temperature"];
  value = data["hydrogen"];
  per = data["presure"];
  alt = data["Altitude"];
 // lum = data["Light"];
  lat = data["latitude"];
  lon = data["longitude"]; 
  soil= data["soil"];
  aud= data["Audio"];
  // Print
  /*
  Serial.print("Hum :");
  Serial.print(hum);
  Serial.print(" Tem :");
  Serial.print(tem);
  Serial.print(" Hyd :");
  Serial.print(value);
  Serial.print(" Alt :");
  Serial.print(alt);
  Serial.print(" Lat :");
  Serial.print(lat);
  Serial.print(" Lon :");
  Serial.println(lon);*/
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different

  ThingSpeak.setField(1, tem);
  ThingSpeak.setField(2, hum);
  ThingSpeak.setField(3, per);
  ThingSpeak.setField(4, alt);
  ThingSpeak.setField(5, value);
  ThingSpeak.setField(6, soil);
  ThingSpeak.setField(7, aud);
  ThingSpeak.setLatitude(lat);
  ThingSpeak.setLongitude(lon);


  
  // write to the ThingSpeak channel
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  delay(15000); // Wait 20 seconds to update the channel again
}
