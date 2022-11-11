/*
const int ledPin=13;//the anode of the LED connects to digital pin D13 of the arduino
const int AOUTpin=0;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=9;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
int limit;
int value;

void setup() {
Serial.begin(9600);//sets the baud rate
pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
pinMode(ledPin, OUTPUT);//sets the pin as an output of the arduino
}

void loop()
{
value= analogRead(AOUTpin);//reads the analaog value from the hydrogen sensor's AOUT pin
limit= digitalRead(DOUTpin);//reads the digital value from the hydrogen sensor's DOUT pin
Serial.print("Hydrogen value: ");
Serial.println(value);//prints the hydrogen value
Serial.print("Limit: ");
Serial.print(limit);//prints the limit reached as either LOW or HIGH 
delay(10);
if (limit == HIGH){
digitalWrite(ledPin, HIGH);//if limit has been reached, LED turns on as status indicator
}
else{
digitalWrite(ledPin, LOW);//if threshold not reached, LED remains off
}
}
*/
#include "DHT.h"

#define DHTPIN 4     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);
float RS_gas = 0;
float ratio = 0;
float sensorValue = 0;
float sensor_volt = 3;
float R0 = 48300.0;
const int DOUTpin=9;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
int limit;
float val = 1;
float res=0; 
void setup() {
 Serial.begin(9600);
 pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
 dht.begin();
}
 
void loop() {
      float h = dht.readHumidity();
  float t = dht.readTemperature();
   float tem;
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("  Temperature: "); 
    Serial.print(t);
    Serial.print(" *C");
   
  }
  limit= digitalRead(DOUTpin);//reads the digital value from the hydrogen sensor's DOUT pin
  if (limit == 0){
    val = 2*val;
  }
  else{
   val = 1;
   tem = t;
  }
   float sensVal = constrain(val, 1, 1000);  // limits range of sensor values to between 10 and 150
   sensorValue = analogRead(A0);
   sensor_volt = sensorValue/1024*5.0;
   RS_gas = (5.0-sensor_volt)/sensor_volt;
   ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
   float x = 1538.46 * ratio;
   float ppm = pow(x,-0.709);
   res = ppm*sensVal;
   Serial.print(" PPM: ");
   Serial.println(res);

  if(res > 50){
    float old_t = tem;
    float new_t = t;
 float dif = (new_t - old_t);
    if(dif > 2){
      Serial.println("fire detected");
    }
  }
   delay(1000);
}
