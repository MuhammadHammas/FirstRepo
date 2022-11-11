#include <avr/sleep.h>
#include <avr/power.h>
#define LED_PIN 13
int A_PIN = A0;
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(A_PIN, INPUT);
  Serial.begin(9600);
  //Save Power by writing all Digital IO LOW - note that pins just need to be tied one way or another, do not damage devices!
  for (int i = 0; i < 20; i++) {
    pinMode(i, OUTPUT);
  }
  
  
  
  //SETUP WATCHDOG TIMER
WDTCSR = (24);//change enable and WDE - also resets
WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
WDTCSR |= (1<<6);//enable interrupt mode


  

}

void loop() {
  ADCSRA |= (1 << 7);
    // read the value from the sensor:
  int sensorValue = analogRead(A_PIN);
Serial.println(sensorValue);
    if (sensorValue > 1000){
      Serial.println("LED");
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
    }
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

void digitalInterrupt(){
  //needed for the digital input interrupt
}

ISR(WDT_vect){
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt
