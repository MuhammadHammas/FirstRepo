#include <TheThingsNetwork.h> // For comunication with things network
/*Wiring of the senssors
 * Air Quality        = A1
 * Soil moisture      = A0
 * Temperature        = D4
 * Noise Sensor       = A3
 * Sun Light          = I2C  ADD = 
 * CO Sensor          = A2
 * GPS Air 530z       = D8,D9
 */
////////////////////LOW POWER///////////////////////////////
// these are the 'standard' register configurations 
byte _SMCR = 0; // disable sleep mode
byte _ADCSRA = 0b10000111; // re-enable ADC
///////////////////GPS//////////////////////////////////////
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define GPS_BAUD  9600
#define ARDUINO_GPS_TX 9 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_RX 8 // GPS RX, Arduino TX pin
TinyGPSPlus gps;
SoftwareSerial gpSerial(ARDUINO_GPS_RX, ARDUINO_GPS_TX); // Create a SoftwareSerial for the GPS
double latitude, longitude, speed=0.0, alt;
static char lati[10], longi[10], vitesse[10], altitude[10];
char data[50];
////////////////////////MQ2/////////////////////////////////
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (2)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

/////////////// Air quality ////////////////////////////////
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A1);
int current_quality =-1;
///////////...............DHT22..................///////////
#include "DHT.h"
#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
//////////////........Acoistic.........////////////////////
int microphonePin = A3;
////////////..........sun light sensor........../////////////////
#include <Wire.h>

#include "Arduino.h"
#include "SI114X.h"

SI114X SI1145 = SI114X();
////////////............Soil Humidity.............////////////////////////
const int AirValue = 610;   //you need to replace this value with Value_1
const int WaterValue = 270;  //you need to replace this value with Value_2
uint32_t soilMoistureValue = 0;
uint32_t soilmoisturepercent=0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set your AppEUI and AppKey
// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *devAddr = "260B3F68";
const char *nwkSKey = "8B8AEC4E5E855800DAD14B82E0590796";
const char *appSKey = "C354C95A74BB2726E8AEBA034543CA37";

#define loraSerial Serial1
#define debugSerial Serial
// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
int count = 0;

void setup()
{  
  gpSerial.begin(GPS_BAUD); 
  loraSerial.begin(57600);
    debugSerial.begin(9600);
 debugSerial.println("-- JOIN");
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  debugSerial.println("-- PERSONALIZE");
  ttn.personalize(devAddr, nwkSKey, appSKey);

  debugSerial.println("-- STATUS");
  ttn.showStatus();

  /////////// MIC///////////////
  pinMode(microphonePin, INPUT);
  ///////// sun light///////////
    uint8_t conf[4];
    Wire.begin();
 

    if (!SI1145.Begin()) {
        debugSerial.println("Si1145 is not ready!");
        delay(1000);
    }
     debugSerial.println("Si1145 is ready!");
//  .............................//
    //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6);//enable interrupt mode

}

void loop()
{
//////////////////////////////Wake up////////////////////////////////////
  wakeUp(); /// for wake up all 
   byte data[34];
   int set = 0;
/////////////////////////////////////DHT22///////////////////////////////
  uint32_t h = dht.readHumidity()*100;
  int t = dht.readTemperature()*100;
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  }
///////////////////////////Audio pitch analysis//////////////////////////
  int mun = 1024;
  int mux = 0;
  int cutting = 0;
  for(int i=0; i<100; i++){
    int val = analogRead(microphonePin);
    mun = min(mun, val);
    mux = max(mux, val);
  }
  uint32_t aud = (mux - mun)*(0.25);
  if(aud >= 150){
    cutting = 3;
  } 
/////////////////////////////Air Quality ////////////////////////////////
     int quality = sensor.slope();    
    uint32_t air = (sensor.getValue())*100;

/////////////////////////////////MQ2////////////////////////////////////
   int mq = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   uint32_t smoke = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
    if (smoke > 20 || mq > 50)
    {   
      int dif_t = (t - old_temp);
      if (dif_t > 1)
      {
        set = 3;
      }
      old_temp = t;
    }
    else if(smoke < 20)
    {
      old_temp = 0;
       set = 0;
    }
//////////////////////////////Sun Light sensor////////////////////////////

    uint32_t uv = (SI1145.ReadUV() / 100);
    uint32_t vis = (SI1145.ReadVisible());
    uint32_t ir = (SI1145.ReadIR());
/////////////////////////////Moisture sensor////////////////////////////
   uint32_t soil= 0 ;
  soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  soil = constrain(soilmoisturepercent, 100, 0);

/////////////////////////////GPS///////////////////////////////////////
  gpSerial.listen();
  latitude = gps.location.lat();
  longitude = gps.location.lng();
  speed = gps.speed.kmph();
  alt = gps.altitude.meters();
  float lati = latitude;
  float longi = longitude;
  long lng = longi*100000;
  long lat = lati*100000;
  uint32_t alti = alt;
  smartDelay(3000);
  
/////////..........||||||||| Data Sending |||||||||................////////////////
//........................Moisture..........................
  data[0] = highByte(soil);
    data[1] = lowByte(soil);
//.................DTH 22..................
  data[2] = highByte(t);
    data[3] = lowByte(t);
  data[4] = highByte(h);
    data[5] = lowByte(h);
//................... MQ2.................
  data[6] = highByte(mq);
    data[7] = lowByte(mq);
//...................ligth.................
  data[8] = highByte(uv);
    data[9] = lowByte(uv);
  data[10] = highByte(vis);
    data[11] = lowByte(vis);
  data[12] = highByte(ir);
    data[13] = lowByte(ir);
//..................Air....................
  data[14] = highByte(air);
    data[15] = lowByte(air);
//.................Pitch....................
if(aud < 90){
      data[16] = highByte(0);
    data[17] = lowByte(0);
}

//..................GPS.......................
/////////...........Altitude.........///////////
    data[18] = highByte(alti);
    data[19] = lowByte(alti);
//////////..........Longitude........../////////
  data[20] = (byte) ((lng & 0xFF000000) >> 24 );
  data[21] = (byte) ((lng & 0x00FF0000) >> 16 );
  data[22] = (byte) ((lng & 0x0000FF00) >> 8  );
  data[23] = (byte) ((lng & 0X000000FF)       );
//////////..........Latitude..........//////////
  data[24] = (byte) ((lat & 0xFF000000) >> 24 );
  data[25] = (byte) ((lat & 0x00FF0000) >> 16 );
  data[26] = (byte) ((lat & 0x0000FF00) >> 8  );
  data[27] = (byte) ((lat & 0X000000FF)       );
////////////////////Alerts///////////////////////
      data[28] = highByte(smoke);
    data[29] = lowByte(smoke);
      data[30] = highByte(cutting);
    data[31] = lowByte(cutting); 
          data[32] = highByte(set);
    data[33] = lowByte(set); 

/////////////////////////////////////////////////////////////////////////////////  

// 10 mint delay
 if (count == 130 || set == 120){
   // ttn.wake();
    ttn.sendBytes(data, sizeof(data)); // sending the data
    delay(30000);
    count = 0;
 }
  count = count +1;
//////////////////////// Serial Monitor///////////////////////////////
//Printing the data into serial monitor
/*
  debugSerial.print("Moist: ");
  debugSerial.print(soil);
  debugSerial.print(" Temp: ");
  debugSerial.print(t/100);
  debugSerial.print(" Hum: ");
  debugSerial.print(h/100);
  debugSerial.print(" CO: ");
  debugSerial.print(mq);
  debugSerial.print(" UV: ");
  debugSerial.print(uv);
  debugSerial.print(" Vis: ");
  debugSerial.print(vis);
  debugSerial.print(" IR: ");
  debugSerial.println(ir);
  debugSerial.print(" Air: ");
  debugSerial.print(air);
  debugSerial.print(" Aud: ");
  debugSerial.print(aud);
  debugSerial.print(" Latitude : ");
  debugSerial.print(latitude, 6);
  debugSerial.print("  Longitude : ");
  debugSerial.print(longitude, 6);
  debugSerial.print(" Altitude : ");
  debugSerial.print(alt);
  debugSerial.print(" Wind : ");
  debugSerial.print(speed);
  debugSerial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
  debugSerial.print( "ppm" );
  debugSerial.print("    ");   
  debugSerial.print("SMOKE:"); 
  debugSerial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
  debugSerial.print( "ppm" );
  debugSerial.print("\n");
*/
////////////////////////////////// Sleep mode ///////////////////////////////////
// uint32_t mseconds = 60000;
// ttn.sleep(mseconds);// Lora Sleep Mode 10 minutes
 
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep

}


///////// function for delay //////////////////////////////
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpSerial .available())
      gps.encode(gpSerial .read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}
/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
void wakeUp()
{
  SMCR = _SMCR; // disable sleep mode
  ADCSRA = _ADCSRA; // re-enable ADC
}

ISR(WDT_vect){
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt
