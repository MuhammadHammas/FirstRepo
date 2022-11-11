#include <TinyGPS.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <DHT.h>
//***************************************************************************************//
// Temperature Sensor                                                                    //
//***************************************************************************************//
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);
//***************************************************************************************//
// MQ 7 gas Sensor                                                                       //
//***************************************************************************************//
const int ledPin=13;//the anode of the LED connects to digital pin D13 of the arduino
const int AOUTpin=0;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=8;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
int limit;
int value;
//***************************************************************************************//
// GPS Module                                                                            //
//***************************************************************************************//
//long   lat,lon; // create variable for latitude and longitude object
float lat,lon ;
TinyGPS gps; // create gps object
//***************************************************************************************//
// Gyro scope                                                                            //
//***************************************************************************************//
MPU6050 mpu;
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
}
String str;
String str1;


void setup()
{
  Serial2.begin(115200);
  Serial1.begin(9600);
  Serial.println("The GPS Received Signal:");
  Serial.begin(57600); // connect gps sensor
  dht.begin();
  pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
  pinMode(ledPin, OUTPUT);//sets the pin as an output of the arduino
  Serial.println("Initialize MPU6050");

  if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
  /* Initialise the sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } 
  /* Setup the sensor gain and integration time */
  configureSensor();
}
 
void loop(){
//    if(Serial1.available()){ // check for gps data
    if(gps.encode(Serial1.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
   }
 //  }
    float one = lat;
   Serial.print(one,6);
   float two = lon;
      Serial.print(" , ");
   Serial.println(two,6);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  value= analogRead(AOUTpin);//reads the analaog value from the hydrogen sensor's AOUT pin
  limit= digitalRead(DOUTpin);//reads the digital value from the hydrogen sensor's DOUT pin
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
//   sensors_event_t event;
  // tsl.getEvent(&event);


 str =String(t)+String(":")+String(h)+String(":")+String(value)+String(":")+String(bmp.readPressure())+String(":")+String(bmp.readAltitude(1013.25))+String(":")+String(one)+String(":")+String(two);
  Serial2.println(str);
}  
