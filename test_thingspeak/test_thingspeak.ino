// #include<EthernetClient.h> //Uncomment this library to work with ESP8266
 #include<ESP8266WiFi.h> //Uncomment this library to work with ESP8266

#include<SPI.h> // Comment this to work with ESP8266 board


char jsonBuffer[500] = "["; // Initialize the jsonBuffer to hold data

char ssid[] = "H"; //  Your network SSID (name)
char pass[] = "Audionic@1234"; // Your network password
WiFiClient client; // Initialize the WiFi client library

char server[] = "api.thingspeak.com"; // ThingSpeak Server


/* Collect data once every 15 seconds and post data to ThingSpeak channel once every 2 minutes */
unsigned long lastConnectionTime = 0; // Track the last connection time
unsigned long lastUpdateTime = 0; // Track the last update time
const unsigned long postingInterval = 120L * 1000L; // Post data every 2 minutes
const unsigned long updateInterval = 15L * 1000L; // Update once every 15 seconds

void setup() {
  Serial.begin(9600);
  // Attempt to connect to WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    delay(10000);  // Wait 10 seconds to connect
  }
  Serial.println("Connected to wifi");
}

void loop() {
  // If update time has reached 1 second, then update the jsonBuffer
  if (millis() - lastUpdateTime >=  updateInterval) {
    updatesJson(jsonBuffer);
  }
    
}
