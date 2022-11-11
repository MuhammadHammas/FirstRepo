//////////////........Acoistic.........////////////////////
int microphonePin = A0;
void setup() {
  // put your setup code here, to run once:
  /////////// MIC///////////////
  pinMode(microphonePin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
///////////////////////////Audio pitch analysis//////////////////////////
  int mun = 1024;
  int mux = 0;
  for(int i=0; i<100; i++){
    int val = analogRead(microphonePin);
    mun = min(mun, val);
    mux = max(mux, val);
  }
  int aud = mux - mun;
  Serial.println(aud);
  if(aud > 150){
    Serial.println("Tree Cutting Detected");
    delay(100);
  }
  delay(200);
}
