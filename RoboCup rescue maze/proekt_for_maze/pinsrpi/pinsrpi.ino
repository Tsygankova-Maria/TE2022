#include <Arduino.h>                                             
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0
#define MAXSPEED 55

#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring( 0, 12 );
#endif
long  Encoder1, Encoder2;

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }else{
    Encoder_2.pulsePosPlus();
  }
}

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) loop();
}

void setup() {
  // put your setup code here, to run once:
  /*pinMode(3,INPUT);
  pinMode(2,INPUT);
  pinMode(7,INPUT);
  pinMode(6,INPUT);*/
  for(int e=A0;e<=A15;++e) pinMode(e,INPUT);
  
  Serial.begin(9600);
 while(true){//A8 A13 A6 A11
  
  #ifdef MeAuriga_H
    led_ring.setpin( 44 );
  #endif
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  
  int rpi2=A13, rpi3=A6, rpi4=A11;
  led_ring.show();
  int sum=digitalRead(rpi2)+digitalRead(rpi3)*2; 
  Serial.print(digitalRead(A8));
  Serial.print(" ");
  Serial.print(digitalRead(A13));
  Serial.print(" ");
  Serial.print(digitalRead(A6));
  Serial.print(" ");
  Serial.println(digitalRead(A11));
/*
   if((sum>0) && (digitalRead(rpi4)==0)){    
       for(int y = 1; y <= 10; y++){
        for ( int i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 255, 0);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }        
     } */
 }
 _loop();
}
void loop() {
  _loop();
  // put your main code here, to run repeatedly://A13 and A8 - button
}
