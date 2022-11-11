#include <Console.h>
#include<SoftwareSerial.h>
#include <UbidotsYUN.h>
#include <TinyGPS.h>
#include "DHT.h"
/****************************************
 * Define Constants
 ****************************************/
#define TOKEN "BBFF-JqseFVjk5LvoXDavBcGDLmyw8LQ3BB"
#define VARIABLE_1 "temperature"  // Change for your variable label desired
#define VARIABLE_2 "Humidity"  // Change for your variable label desired
#define VARIABLE_3 "CO Level(PPh)"  // Change for your variable label desired
#define VARIABLE_4 "position"  // Change for your variable label desired
#define VARIABLE_5 "Longitude"  // Change for your variable label desired
Ubidots client(TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

//Temperature and humidity sensor


#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);
////////////****************************///////////////////////////////////////////||||||
//CO MQ7 sensor                                                                     |||||
//////////////***********************************//////////////////////////////////||||||
const int AOUTpin=0;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=8;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
int limit;
int value;
////////////****************************///////////////////////////////////////////||||||
//GPS                                                                               |||||
//////////////***********************************//////////////////////////////////||||||
//long   lat,lon; // create variable for latitude and longitude object
float lat,lon;
TinyGPS gps; // create gps object
SoftwareSerial mySerial(3, 4); // RX, TX
char str_late[15];
char str_lng[15];
///////////////*************************************////////////////////////////////////////
/****************************************
 * Main Functions
 ****************************************/

void setup() {

  // Initialize Console and wait for port to open:
  client.init();
  mySerial.begin(9600);
  Serial.begin(9600);
    // Initialize Console and wait for port to open:
  Bridge.begin();
 // Console.begin();
   Serial.println("DHTxx test!");
  dht.begin();
  pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino

  Console.begin();

  // Wait for Console port to connect

  while (!Console);

  Console.println("Hi, what's your name?");
}

void loop() {


      Console.print("Hi ");
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    /*
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C");*/
  }
  value= analogRead(AOUTpin);//reads the analaog value from the hydrogen sensor's AOUT pin
  limit= digitalRead(DOUTpin);//reads the digital value from the hydrogen sensor's DOUT pin
  /*
  Serial.print("Hydrogen value: ");
  Serial.println(value);//prints the hydrogen value
  Serial.print("Limit: ");
  Serial.print(limit);//prints the limit reached as either LOW or HIGH 
  delay(100);
  if (limit == HIGH){
  digitalWrite(ledPin, HIGH);//if limit has been reached, LED turns on as status indicator
  }
  else{
  digitalWrite(ledPin, LOW);//if threshold not reached, LED remains off
  }*/
    if(mySerial.available()){ // check for gps data
    if(gps.encode(mySerial.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
/*
    Serial.print("Position: ");
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);
    
    Serial.print(",");
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.println(lon,6); 
  */ 
   }
  }
  char context[50];
  
  float late = lat; // Assign the Latitude desired
  float lng = lon; // Assign the Longitude desired
  
  /* float value is copied onto str_lat/str_lng */
  dtostrf(late, 4, 6, str_late);
  dtostrf(lng, 4, 6, str_lng);
  
  /* Build the coordinates context to be sent*/
  sprintf(context, "lat=%s\\$lng=%s", str_late, str_lng);
  /* Send variable with coordinates context to Ubidots*/
  client.add(VARIABLE_4, 1, context);
  client.add(VARIABLE_1, t); // Change for your variable name
  client.add(VARIABLE_2, h); // Change for your variable name
  client.add(VARIABLE_3, value); // Change for your variable name
  client.sendAll();
  delay(1000);
}
