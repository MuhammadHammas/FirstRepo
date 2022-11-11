//////////////////////////// Pin configrations//////////////////////////////
//          Temperature(DHT-22) = D2
//          Presure(BMP-280)    = SCL, SDA
//          CO(MQ-7)            = A1, D8
//          Air Quality(MQ-135) = A2
//          Soil Moisture       = A3
//          Noise(mic)          = A8
//          GPS(neo-6)          = Serial1
//          ESP-8266            = Serial2
///////////////////////////////////////////////////////////////////////////
#include <ArduinoJson.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
//***************************************************************************************//
// Temperature Sensor                                                                    //
//***************************************************************************************//
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);
//***************************************************************************************//
// MQ 7 gas Sensor                                                                       //
//***************************************************************************************//
const int ledPin=13;//the anode of the LED connects to digital pin D13 of the arduino
const int AOUTpin=1;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=8;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino
float RS_gas = 0;
float ratio = 0;
float sensorValue = 0;
float sensor_volt = 3;
float R0 = 48300.0;
int limit;
float val = 1;
float res=0; 

//***************************************************************************************//
// MQ 135 gas Sensor                                                                       //
//***************************************************************************************//
int sensorValue1;
int digitalValue;


//***************************************************************************************//
// GPS Module                                                                            //
//***************************************************************************************//
//long   lat,lon; // create variable for latitude and longitude object
float lati,lon ;
TinyGPS gps; // create gps object
//***************************************************************************************//
// BMP280 Pressure and altitude                                                          //
//***************************************************************************************//

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C


//***************************************************************************************//
// Soil Humidity                                                                         //
//***************************************************************************************//
const int AirValue = 610;   //you need to replace this value with Value_1
const int WaterValue = 270;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent=0;
//////////////........Acoistic.........////////////////////
int microphonePin = A8;
//........................................................//


void setup() 
{
  Serial.begin(57600);
  Serial1.begin(9600);
   /////////// MIC///////////////
  pinMode(microphonePin, INPUT);
  //////////////////////////////
  dht.begin();
  pinMode(AOUTpin, INPUT);//sets the pin as an input to the arduino
  pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
  pinMode(ledPin, OUTPUT);//sets the pin as an output of the arduino

    Serial.println(F("BMP280 test"));
  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}
                  
void loop()
{

////////////////////// GPS ////////////////////////////////////////////////
    if(Serial1.available()){ // check for gps data
    if(gps.encode(Serial1.read()))// encode gps data
    { 
    gps.f_get_position(&lati,&lon); // get latitude and longitude

   }
  }
////////////////////////////// Temperature ///////////////////////////////////////
  float h = dht.readHumidity();
  float t = dht.readTemperature();
     float tem;
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {

  }
////////////////////////////////////////// CO sensor ///////////////////////////////////////////
    limit= digitalRead(DOUTpin);//reads the digital value from the hydrogen sensor's DOUT pin
  if (limit == 0){
    val = 2*val;
  }
  else{
   val = 1;
   tem = t;
  }
   float sensVal = constrain(val, 1, 1000);  // limits range of sensor values to between 10 and 150
   sensorValue = analogRead(AOUTpin);
   sensor_volt = sensorValue/1024*5.0;
   RS_gas = (5.0-sensor_volt)/sensor_volt;
   ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
   float x = 1538.46 * ratio;
   float ppm = pow(x,-0.709);
   res = ppm*sensVal;

  if(res > 50){
    float old_t = tem;
    float new_t = t;
 float dif = (new_t - old_t);
    if(dif > 2){
      Serial.println("fire detected");
    }
  }
////////////////////////////////// Air Quality ///////////////////////////////////////////////
  sensorValue1 = analogRead(A2); // read analog input pin A2
  int Aqi = map(sensorValue1, 0, 1023, 0, 100);
//////////////////////////////////////soil/////////////////////////////////////////////////////
  float soil =0 ;
  soilMoistureValue = analogRead(A3);  //put Sensor insert into soil
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  if(soilmoisturepercent >= 100)
  {
    //Serial.println(" 100 %");
    soil = 100;
  }
  else if(soilmoisturepercent <=0)
  {
   // Serial.println(" 0 %");
   soil= 0;
  }
  else if(soilmoisturepercent >0 && soilmoisturepercent < 100)
  {
   // Serial.print(soilmoisturepercent);
   // Serial.println(" % ");
   soil = soilmoisturepercent;
  }
///////////////////////////Audio pitch analysis//////////////////////////
  int mun = 1024;
  int mux = 0;
  for(int i=0; i<100; i++){
    int val = analogRead(microphonePin);
    mun = min(mun, val);
    mux = max(mux, val);
  }
  float aud = mux - mun;

/////////////////////////////////////////////////////////////////////////////
  Serial.println();
  Serial.print("Hum :");
  Serial.print(h);
  Serial.print(" Tem :");
  Serial.print(t);
  Serial.print(" CO: ");
  Serial.print(res);
  Serial.print(" Air: ");
  Serial.print(Aqi);
  Serial.print(" Moisture :");
  Serial.print(soil);
  Serial.print(" Pre :");
  Serial.print(bmp.readPressure());
  Serial.print(" Alt :");
  Serial.print(bmp.readAltitude(1013.25));
  Serial.print(" Lat :");
  Serial.print(lati,6);
  Serial.print(" Lon :");
  Serial.println(lon,6);

  delay(500);
   }
