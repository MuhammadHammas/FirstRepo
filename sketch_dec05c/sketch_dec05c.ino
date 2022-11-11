
/*
void setup() {
// put your setup code here, to run once:

Serial.begin(115200);
while (!Serial) {
; 
}
}

void loop() {
// put your main code here, to run repeatedly:
if (Serial.available()) {
  Serial.write(Serial.read()); 
}
}*/
// Robo India Tutorial 
// Digital Input and Output on LED 
// Hardware: NodeMCU

void setup() {
  Serial.begin(9600);                  
  Serial.print("Hello World");  
}
void loop() {
    Serial.println("Hello World");
 }
