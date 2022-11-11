double latitude, longitude, speed=0.0, alt;
static char lati[10], longi[10], vitesse[10], altitude[10];

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
     byte data[2];
  // put your main code here, to run repeatedly:
latitude = 30.237371;
longitude = -66.932859;
float lati = latitude ;
float longi = longitude; 
/*
Serial.print(" Latitude : ");
Serial.print(latitude,6);
Serial.print(" Longitude : ");
Serial.println(longitude,6);
*/
Serial.print(" Latitude : ");
Serial.print(lati,6);
Serial.print(" Longitude : ");
Serial.println(longi, 6);
int soil = longi*100;
/*
long lng = longi*100000;
byte payload[4];
payload[0] = (byte) ((lng & 0xFF000000) >> 24 );
payload[1] = (byte) ((lng & 0x00FF0000) >> 16 );
payload[2] = (byte) ((lng & 0x0000FF00) >> 8  );
payload[3] = (byte) ((lng & 0X000000FF)       );
*/
   data[0] = highByte(soil);
   data[1] = lowByte(soil);
float soil_humidity = (data[0]<<8 |data[1]);
      soil_humidity = soil_humidity/100;
//float myVal = ((int)(data[0]) << 8)+ data[1];
 // myVal= myVal/100;
//float myVal = ((long)(payload[0]) << 24)+ ((long)(payload[1]) << 16)+ ((long)(payload[2]) << 8)+ ((long)(payload[3]));
//myVal = myVal/100000;
Serial.println(soil_humidity);
}
