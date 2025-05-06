#include <Arduino.h>                                             
#include <Wire.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27,16,2); 
#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0
#define MAXSPEED 220
#include <FIFObuf.h>


FIFObuf<int> FIFA(30);
long er;
int t = 5;
int y;
int i;
int w;
int storona;
int angel;
int gyro;
int lastgyro;
int diff;
float dis_front;
float dis_left;
float dis_right;
float dis_left_b;//A15
float dis_right_b;//A10
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int htColorSensorAddress = 0x01; //we got this from I2C Scanner
int TestValue;
byte ColorRed, ColorGreen, ColorBlue;
float H,S,V;
float BlackV = 0.3;
float BlueH = 215;
long  Encoder1, Encoder2;
float current_1, current_2;
int stena=17;
bool vict=1;
bool black_pol=1, puti=1;
#define min2(a, b) (a < b ? a : b)
#define min3(a, b, c) (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c)
#define max2(a, b) (a > b ? a : b)
#define max3(a, b, c) (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c)
#define clip(a, b, c) min2(c, max2(b, a))

MeGyro gyro_0(0, 0x69);
MeLightSensor lightsensor_12(12);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeColorSensor colorsensor_7(7);

Servo servo1;
#ifdef MeAuriga_H
MeRGBLed led_ring( 0, 12 );
#endif
void rotate_left(){
 move(3, 85/ 100.0 *255);//65
}
void rotate_right(){
 move(4, 85 / 100.0 * 255);//65
}
void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}
float dis_front_read(void);

struct myNode{
uint8_t wall=0;
//char tile=' ';
//bool victim=0;
uint8_t count=0; 
//int coordinata_x, coordinata_y; 
};
myNode mymap[30][30];

int cord_x=15, cord_y=25;
int napravlenie=2;

uint8_t tam_l,tam_r;
uint8_t x_left, x_front, x_right, x_back, y_left, y_front, y_right, y_back;

void controlLight(MeColorSensor colorSensor, uint8_t lightState){
  if(lightState == 0){
    colorSensor.TurnOffLight();
  }else{
    colorSensor.TurnOnLight();
  }
}

uint8_t detectedColor(MeColorSensor colorSensor, uint8_t colorType){
  if(colorType == colorSensor.Returnresult()){
    return 1;
  }
  return 0;
}
void examination(){
 if(dis_front < 5 || dis_front > 50){
  dis_front = 50;
 }
 if(dis_left < 0 || dis_left > 50){
  dis_left = 50;
 }
 if(dis_right < 0 || dis_right > 50){
  dis_right = 50;
 }
 if(dis_left_b < 0 || dis_left_b > 50){
  dis_left_b = 50;
 }
 if(dis_right_b < 0 || dis_right > 50){
  dis_right_b = 50;
 }
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
float IR(int p){
 float volts = analogRead(p);
 return 4200 / (volts - 57);
}
void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if(direction == 1){ //nazad
    leftSpeed = -speed;
    rightSpeed = speed;
  }else if(direction == 2){ //vpered
    leftSpeed = speed;
    rightSpeed = -speed;
  }else if(direction == 3){//left
    leftSpeed = -speed;
    rightSpeed = -speed;
  }else if(direction == 4){//right 
    leftSpeed = speed;
    rightSpeed = speed;
  }
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
}

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) loop();
}


void gyro_rotate(int side){ // 0 - поворот налево, 1 - поворот направо, 2 - поврот прямо(проезд прямо)
  if(side == 0){
     led_ring.setColor( 12, 0, 0, 100);
     led_ring.show();
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.075);
     dis_front_read();
     dis_left =  IR(A15);
     dis_right =  IR(A10);
     examination();
     if(dis_right <= stena) storona = 1;
     else storona = 0;
     if(dis_front < stena){
      move(2,150);
      _delay(1);
      move(1,0);
      _delay(0.095);
      move(1,64);     
      _delay(0.5);    
       move(3,0);
       _delay(0.5);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     if(gyro > -90){
      angel = lastgyro - 90;
      while(angel - gyro < 0){
       if(vict)rpi();
       move(1,0);    
       rotate_left();
       //antizalip();
       _delay(0.095);
       move(1,0);
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro - 90;
      while(gyro > -180 && -90 >= gyro || gyro > angel + 360){
        if(vict)rpi();
       move(1,0);   
       rotate_left();
       //antizalip();
       _delay(0.095);
       move(1,0); 
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);        
      }
     }
      move(1,0);
     _delay(0.2);
     gyro_0.update();
     gyro = gyro_0.getAngle(3);
     if(lastgyro > -90){
      while(abs(gyro)-abs(angel) > 0){
       if(vict)rpi();
       move(1,0);     
       rotate_right();
       //antizalip();
       _delay(0.095);
       move(1,0); 
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else if(180 >= gyro && gyro >= 90 ){
      while(360 + angel - gyro > 0){
       if(vict)rpi();
       move(1,0);  
       rotate_right();
       //antizalip();
       _delay(0.095);
       move(1,0);
       _delay(0.01);   
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
    move(1,100);
     _delay(0.4);
     move(1,0);
     if(storona == 1){
      move(1,150);
      _delay(0.75);
      move(1,0);
      _delay(0.075);        
      storona = 0;
     }
    if(vict) rpi();
    napravlenie--;
    if(napravlenie==0) napravlenie=4;
    move(1,0);
    delay(250);    
    //alignment();  
    vict=1;

    led_ring.setColor( 12, 0, 0, 0 );
    led_ring.show();
  }
  
  if(side == 1){
     led_ring.setColor( 6, 0, 0, 100);
     led_ring.show();
     dis_front_read();
     dis_left =  IR(A15);
     dis_right =  IR(A10);
     examination();
     if(dis_left <= stena) storona = 1;
     else storona = 0;
     if(dis_front < stena){
      move(2,150);
      _delay(1); 
      move(2,0);
      _delay(0.075);
      move(1,64);
      _delay(0.5);    
       move(3,0);
       _delay(0.5);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     if(gyro < 90){
      angel = lastgyro + 90;
      while(angel - gyro > 0){
        if(vict)rpi();
       move(1,0);  
       rotate_right();
       //antizalip();
       _delay(0.095);
       move(1,0); 
       _delay(0.01); 
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro + 90;
      while(gyro < 180 && 90 <= gyro || gyro < angel - 360){
        if(vict)rpi();
       move(1,0);   
       rotate_right();
       //antizalip();
       _delay(0.095);
       move(1,0);
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);        
      }
     }
      move(1,0); 
     _delay(0.2);
     gyro_0.update();
     gyro = gyro_0.getAngle(3);
     if(lastgyro < 90){
      while(gyro - angel > 0){
        if(vict)rpi();
       move(1,0);    
       rotate_left();
      //antizalip();
       _delay(0.095);
       move(1,0);
       _delay(0.01);       
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else {
      while(360 - angel + gyro > 0){
        if(vict)rpi();
       move(1,0);
       rotate_left();
      //antizalip();
       _delay(0.095);
       move(1,0);
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
    move(1,100);
     _delay(0.4);
     move(1,0);
    if(storona == 1){
      move(1,150);
      _delay(0.75);
      move(1,0);
      _delay(0.075);        
      storona = 0;
     }
     if(vict) rpi();
     napravlenie++;
     if(napravlenie==5) napravlenie=1;
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);   
     delay(250); 
    // alignment();  
     vict=1;

     led_ring.setColor( 6, 0, 0, 0 );
     led_ring.show();

    }
    
    if(side == 2){  
    black_pol=1;
    dis_front_read();
     examination();
    Encoder1=Encoder_1.getCurPos();
    Encoder2=Encoder_2.getCurPos();
     gyro_y();
    while(abs(Encoder_1.getCurPos()-Encoder1)<380 && abs(Encoder_2.getCurPos()-Encoder2)<380){  
      move(2,140);
   // antizalip();
     /* if(current_1==Encoder_1.getCurPos() && current_2==Encoder_2.getCurPos()){
      move(1,100);
      _delay(0.2);
      move(1,0);
    }*/
     _delay(0.01);
     HSV();
     if(vict) rpi();
       gyro_0.update();
     if(gyro_0.getAngle(1)<3 || gyro_0.getAngle(1)>-3)//antizalip();
     if(BlackV > V){
        move(2,0);
        abyss();
        black_pol=0;
        break;}
     _delay(0.01); 
     printl();
  current_1=Encoder_1.getCurPos();
  current_2=Encoder_2.getCurPos();
     
    }
     alignment();
     _delay(0.2);
     dis_left =  IR(A15);
     dis_right =  IR(A10);
     examination();
     vict=1;
     
  if(black_pol){
     if(napravlenie==1) cord_x--;
     else if(napravlenie==2) cord_y--;
     else if(napravlenie==3) cord_x++;
     else cord_y++;  
      mymap[cord_x][cord_y].count+=1;
    }
    abyss();
   }
}

void HSV(){
  ReadRGBSensor();
  RGBtoHSV((float) ColorRed, (float) ColorGreen, (float) ColorBlue, H, S, V); 
}

void ReadRGBSensor(){
  Wire.beginTransmission(htColorSensorAddress);
  Wire.write(0x43); // Red+Green+Blue
  Wire.endTransmission();
  Wire.requestFrom(htColorSensorAddress, 3); 
   while(Wire.available() < 3);
   ColorRed = Wire.read();
   ColorGreen = Wire.read();
   ColorBlue = Wire.read();
}
void RGBtoHSV(float red, float green, float blue, float &hue, float &sat, float &value){
  hue = 0;
  sat = 0;
  value = 0;
  //   Value
  float rgb_max = max3(red, green, blue);
  float rgb_min;
  value = rgb_max / 2.56;
  if (value == 0){
    hue = -1;
    sat = -1;
    return;
  }
  //   Saturation
  red /= rgb_max;
  green /= rgb_max;
  blue /= rgb_max;
  rgb_max = max3(red, green, blue);
  rgb_min = min3(red, green, blue);
  sat = (rgb_max - rgb_min) * 100;
  if (sat == 0){
    hue = -1;
    return;
  }
  //   Hue
  red = (red - rgb_min) / (rgb_max - rgb_min);
  green = (green - rgb_min) / (rgb_max - rgb_min);
  blue = (blue - rgb_min) / (rgb_max - rgb_min);
  rgb_max = max3(red, green,blue);
  rgb_min = min3(red, green,blue);
  if (rgb_max == red){
    hue = 0.0 + 60.0*(green-blue);
    if (hue < 0.0){
      hue += 360.0;
    }
  } else if (rgb_max == green){
    hue = 120.0 + 60.0 * (blue-red);
  } else {
    hue = 240.0 + 60.0 * (red-green);
  }
}
void abyss(){
  HSV();
  rpi();
 if(BlackV > V){
    Encoder1=Encoder_1.getCurPos();
    Encoder2=Encoder_2.getCurPos();

    while(abs(Encoder_1.getCurPos()-Encoder1)<100 && abs(Encoder_2.getCurPos()-Encoder2)<100){
     move(1,150);
     _delay(0.14); 
    }
    move(1,0);
     _delay(2);
     
    dis_left =  IR(A15);
    dis_right =  IR(A10);
     examination();
    if(dis_left > dis_right){
       gyro_rotate(0);
      dis_front_read();
      if(dis_front<stena) gyro_rotate(0);
    }
    else{
       gyro_rotate(1);
      dis_front_read();
      if(dis_front<stena) gyro_rotate(1);
    }

 }
 else if(H>BlueH){
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(5);     
 }
}
void complect(){
 Encoder_1.setTarPWM(0);
 Encoder_2.setTarPWM(0);
 _delay(0.5);
  servo1.write(1);
  delay(2000);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(168);
  delay(2000); 
  _delay(1);
}
void rpi(){ 
  pinMode(A8,INPUT);//rpi1 
  pinMode(A13,INPUT);//rpi2 
  pinMode(A6,INPUT);//rpi3
  pinMode(A11,INPUT);//rpi4
  led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
  led_ring.show();
  int sum=digitalRead(A13)+digitalRead(A6)*2; 
  Serial.println(digitalRead(A13)+digitalRead(A6)*2);
  dis_left =  IR(A15);
  dis_right =  IR(A10);
  examination(); 
 //((dis_left<=stena && digitalRead(A11)==0) || (dis_right<=stena && digitalRead(A11)==1)) && 
   if(millis()-er>=30000 && ((dis_left<=stena && digitalRead(A11)==0) || (dis_right<=stena && digitalRead(A11)==1)) && (digitalRead(A13)+digitalRead(A6)*2>0) && bitRead(mymap[cord_x][cord_y].wall,5)==0){
       move(1,0);
       er=millis();
       vict=0;
       alignment();
       _delay(2);
       for(y = 1; y <= 10; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 255, 0);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }
       //if(digitalRead(A11)==0)gyro_rotate(1);
       //else gyro_rotate(0);
       for(int i =0;i<=sum-2;++i) complect(); 
   }
       if(!vict){
          bitSet(mymap[cord_x][cord_y].wall,5);
          gyro_rotate(2);
       }
       move(1,0); 
}


void alignment(){  
  long time_now;
  float a=0.1, b=0.09;
  dis_left =  IR(A15);
  dis_right =  IR(A10); 
  dis_right_b = IR(A14);
  dis_left_b = IR(A9);
  dis_front_read();
  examination();  
  if(dis_right<=stena && dis_left<=stena){
    if(dis_right-dis_left >=7){
      time_now=millis();
     while(dis_right-dis_left >=7 && millis()-time_now<=3000){
        rotate_right();
        //antizalip();
        _delay(0.2);
    move(1,0);  
        rotate_left();
        _delay(b);
    move(1,0);
      dis_left =  IR(A15);
      dis_right =  IR(A10);
      examination();
      printl();
     }
    }
    else if(dis_left-dis_right >=7){
      time_now=millis();
     while(dis_left-dis_right >=7 && millis()-time_now<=3000){
          rotate_left();
        //antizalip();
        _delay(0.2);
    move(1,0);
        rotate_right();
        _delay(b);
    move(1,0);  
        dis_left =  IR(A15);
        dis_right =  IR(A10);
        examination();
        printl();
     }
    }
  }
  else if(dis_right<=stena){
   /* if(dis_right < stena/1.5){while(dis_right < stena/1.5){
      rotate_left();
      //antizalip();
        _delay(a);
    move(1,0);
        rotate_right();
        _delay(b);
    move(1,0);  
      dis_left = IR(A15);
      examination();
      printl();
    }
    }
    else if(dis_right > stena/1.5){while(dis_right > stena/1.5){
          rotate_right();
           //antizalip();
        _delay(a);
    move(1,0);
        rotate_left();
        _delay(b);  
    move(1,0);
          dis_left = IR(A15);
          dis_left_b = IR(A9);
          examination();
          printl();
        }
    }*/
    if(dis_right-dis_right_b >7){
      time_now=millis();
      while(dis_right-dis_right_b >7 && millis()-time_now<=3000){
        rotate_right();
       //antizalip();
        _delay(a);
    move(1,0);
        rotate_left();
        _delay(b);
    move(1,0);  
        dis_right_b = IR(A14);
        dis_right =  IR(A10);
        examination();
        printl();  
      }
    }
  else if(dis_right_b-dis_right >7){
    time_now=millis();
      while(dis_right_b-dis_right >7 && millis()-time_now==3000){
        rotate_left();
       //antizalip();
        _delay(a);
    move(1,0);
        rotate_right();
        _delay(b);
    move(1,0);  
        dis_right_b = IR(A14);
        dis_right =  IR(A10);
        examination();
        printl();
    }
  }
  }
  else if(dis_left<=stena){
    /*if(dis_left > stena/1.5){while(dis_left > stena/1.5){
      rotate_left();
      //antizalip();
        _delay(a);
    move(1,0);
        rotate_right();
        _delay(b);
    move(1,0);  
      dis_left = IR(A15);
      examination();
      printl();
    }
    }
    else if(dis_left < stena/1.5){while(dis_left < stena/1.5){
          rotate_right();
           //antizalip();
        _delay(a);
    move(1,0);
        rotate_left();
        _delay(b);
    move(1,0);  
          dis_left = IR(A15);
          dis_left_b = IR(A9);
          examination();
          printl();
        }
    }*/
    if(dis_left-dis_left_b < 7 ){
      time_now=millis();
      while(dis_left-dis_left_b < 7 && millis()-time_now<=3000){
        rotate_right();
       //antizalip();
        _delay(a);
    move(1,0);
        rotate_left();
        _delay(b);
    move(1,0);  
          dis_left = IR(A15);
          dis_left_b = IR(A9);
        examination();
        printl();  
      }
    }
  else if(dis_left_b-dis_left < 7){
    time_now=millis();
      while(dis_left_b-dis_left < 7 && millis()-time_now==3000){
        rotate_left();
       //antizalip();
        _delay(a);
    move(1,0);
        rotate_right();
        _delay(b);
    move(1,0);  
          dis_left = IR(A15);
          dis_left_b = IR(A9);
        examination();
        printl();
    }
  }
    }
    move(1,0);
    _delay(0.3);
  }

void gyro_y(){
 gyro_0.update();
 int gyro_y = gyro_0.getAngle(1); 
 
 if(gyro_y < -5){
  move(1,0);
  _delay(0.2);
  dis_left =  IR(A15);
  dis_right =  IR(A10); 
  examination();
  while(gyro_y < -5){
   dis_left =  IR(A15);
   dis_right =  IR(A10); 
    examination();
    Encoder_1.setMotorPwm(170+(dis_right-dis_left)*3); //left
    Encoder_2.setMotorPwm(-170+(dis_right-dis_left)*3);  //right
   _delay(0.02);
 gyro_0.update();
   gyro_y = gyro_0.getAngle(1);  
  }
  //puti=1;
   dis_left =  IR(A15);
   dis_right =  IR(A10); 
    examination();
  if(dis_left>=stena) gyro_rotate(0);
  else gyro_rotate(1);
 } 
 //else if(gyro_y<-4 && gyro_y>-10) 
    //while(gyro_y<-4 && gyro_y>-10) move(2, 180);
 else if(gyro_y > 7){
   dis_left =  IR(A15);
   dis_right =  IR(A10);
    examination();
  while(dis_left<=stena && dis_right<=stena){
   move(2,64);
   dis_left =  IR(A15);
   dis_right =  IR(A10); 
    examination();
 gyro_0.update();
   gyro_y = gyro_0.getAngle(1);
  }
   move(3,0);
   _delay(0.5);
 }
}
void antizalip(){
  current_1=Encoder_1.getCurPos();
  current_2=Encoder_2.getCurPos();
  _delay(0.009);
  
  if(current_1==Encoder_1.getCurPos() && current_2==Encoder_2.getCurPos()){
    move(1,100);
    _delay(0.02);
    move(1,0);
  }
}
void okruga(){
    dis_front_read();
    dis_left =  IR(A15);
    dis_right =  IR(A10);
    dis_left_b = IR(A9);
    dis_right_b = IR(A14);
    examination();    
    if(vict)rpi();

    if(dis_front<=stena) bitSet(mymap[cord_x][cord_y].wall,napravlenie);    
    if(dis_left<=stena && dis_left_b<=stena) bitSet(mymap[cord_x][cord_y].wall,tam_l);
    if(dis_right<=stena && dis_right_b<=stena) bitSet(mymap[cord_x][cord_y].wall,tam_r);
}
void printl(){
    dis_front_read();
    dis_left =  IR(A15);
    dis_right =  IR(A10);
    dis_left_b = IR(A9);
    dis_right_b = IR(A14);
    examination();  
    
     lcd.setCursor(0,0);
     lcd.print("F:");
     lcd.print(dis_front);
     lcd.setCursor(9,0);
     lcd.print("R:");
     lcd.print(dis_right);
     /*
     lcd.setCursor(0,1);
     lcd.print("L:");
     lcd.print(dis_left_b);
     lcd.setCursor(9,1);
     lcd.print("R:");
     lcd.print(dis_right_b);
     */
     lcd.setCursor(0,1);
     lcd.print(cord_x);
     lcd.print(":");
     lcd.print(cord_y);
     lcd.print("   ");
     //gyro_0.update();
     //lcd.print(gyro_0.getAngle(1));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,1));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,2));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,3));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,4));
     //lcd.print(bitRead(mymap[cord_x][cord_y].wall,5));
     lcd.print(' ');
     lcd.print(mymap[cord_x][cord_y].count);
  
}
float dis_front_read(){
 VL53L0X_RangingMeasurementData_t measure;
 lox.rangingTest(&measure, false);
 dis_front = measure.RangeMilliMeter / 10;
 return dis_front;
}

void Back(){
  for(i=1;i<4;i++) 
  if(bitRead(mymap[cord_x][cord_y].wall,i)==0){
    dis_front_read();
    dis_left =  IR(A15);
    dis_right =  IR(A10);
    dis_left_b = IR(A9);
    dis_right_b = IR(A14);
    examination();

    if(i==1 && dis_right>=stena){

     led_ring.setColor( 6, 0, 0, 100);
     led_ring.show();
     _delay(0.4);
     
      if(tam_r==1 && mymap[cord_x-1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x-1);}
      else if(tam_r==2 && mymap[cord_x][cord_y-1].count<=0) {FIFA.push(cord_y-1); FIFA.push(cord_x);}
      else if(tam_r==3 && mymap[cord_x+1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x+1);}
      else if(tam_r==4 && mymap[cord_x][cord_y+1].count<=0) {FIFA.push(cord_y+1); FIFA.push(cord_x);}

      
     led_ring.setColor( 6, 0, 0, 0);
     led_ring.show();
    }
    if(i==2 && dis_left<=stena){
      
      led_ring.setColor( 12, 0, 0, 100);
      led_ring.show();
      _delay(0.4);
      
        if(tam_r==1 && mymap[cord_x-1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x-1);}
        else if(tam_l==2 && mymap[cord_x][cord_y-1].count<=0) {FIFA.push(cord_y-1); FIFA.push(cord_x);}
        else if(tam_l==3 && mymap[cord_x+1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x+1);}
        else if(tam_l==4 && mymap[cord_x][cord_y+1].count<=0) {FIFA.push(cord_y+1); FIFA.push(cord_x);}

    
      led_ring.setColor( 12, 0, 0, 0);
      led_ring.show();
    }
    if(i==3 && dis_front<=stena){
        if(tam_r==1 && mymap[cord_x-1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x-1);}
        else if(napravlenie==2 && mymap[cord_x][cord_y-1].count<=0) {FIFA.push(cord_y-1); FIFA.push(cord_x);}
        else if(napravlenie==3 && mymap[cord_x+1][cord_y].count<=0) {FIFA.push(cord_y); FIFA.push(cord_x+1);}
        else if(napravlenie==4 && mymap[cord_x][cord_y+1].count<=0) {FIFA.push(cord_y+1); FIFA.push(cord_x);}
    }
}
}

void setup() {
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  servo1.attach(9);
  servo1.write(168);
  pinMode(13,OUTPUT);
  delay(2000);
  Wire.begin();
  Serial.begin(9600); 
  gyro_0.begin();
  #ifdef MeAuriga_H
    led_ring.setpin( 44 );
  #endif
  lcd.init();                     
  lcd.backlight();// Включаем подсветку дисплея
  uint16_t result;
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
     
   while(1) {
    lcd.clear();
     printl();
     lcd.setCursor(0,1);
     lcd.print(cord_x);
     lcd.print(":");
     lcd.print(cord_y);
     lcd.print("   ");
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,1));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,2));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,3));
     lcd.print(bitRead(mymap[cord_x][cord_y].wall,4));
     //lcd.print(bitRead(mymap[cord_x][cord_y].wall,5));
     //lcd.print(gyro_0.getAngle(1));
     lcd.print(' ');
     lcd.print(mymap[cord_x][cord_y].count);
       
    int duration, cm; // назначаем переменную "cm" и "duration" для показаний датчика
    gyro_y(); 
  
    Serial.print("DDIIISSS:::   "); Serial.print(dis_front);
     
    Wire.beginTransmission(0x5A);
    Wire.write(0x07);            // sends instruction byte
    Wire.endTransmission(false);     // stop transmitting
    Wire.requestFrom(0x5A, 3);//Send data n-bytes read
    result = Wire.read(); //Receive DATA
    result |= Wire.read() << 8; //Receive DATA
    uint8_t pec = Wire.read();
    Wire.beginTransmission(0x5A);
    Wire.write(0x07);            // sends instruction byte
    Wire.endTransmission(false);     // stop transmitting
    Wire.requestFrom(0x5A, 3);//Send data n-bytes read
    result = Wire.read(); //Receive DATA
    result |= Wire.read() << 8; //Receive DATA


    gyro_y();
    dis_front_read();
    dis_left =  IR(A15);
    dis_right =  IR(A10);
    dis_left_b = IR(A9);
    dis_right_b = IR(A14);
    examination();

    tam_l=napravlenie-1, tam_r=napravlenie+1;
    if(tam_l==0) tam_l=4;
    if(tam_r==5) tam_r=1;
    okruga();
    printl();
    Back();
    rpi();
//    alignment();
    
  if(mymap[cord_x][cord_y].count>1){
      if(bitRead(mymap[cord_x][cord_y].wall,1)==0 && mymap[cord_x-1][cord_y].count==0) {
        puti=0;
        if(tam_r==1) while(napravlenie!=1) gyro_rotate(1);
        else while(napravlenie!=1) gyro_rotate(0);
        puti=1;
      }
      else if(bitRead(mymap[cord_x][cord_y].wall,3)==0 && mymap[cord_x][cord_y-1].count==0){
        puti=0;
        if(tam_r==2) while(napravlenie!=2) gyro_rotate(1);
        else while(napravlenie!=2) gyro_rotate(0);
        puti=1;
      }
      else if(bitRead(mymap[cord_x][cord_y].wall,2)==0 && mymap[cord_x+1][cord_y].count==0){
        puti=0;
        if(tam_r==3) while(napravlenie!=3) gyro_rotate(1);
        else while(napravlenie!=3) gyro_rotate(0);
        puti=1;
      }
      else if(bitRead(mymap[cord_x][cord_y].wall,4)==0 && mymap[cord_x][cord_y+1].count==0){
        puti=0;
        if(tam_r==4) while(napravlenie!=4) gyro_rotate(1);
        else while(napravlenie!=4) gyro_rotate(0);
        puti=1;
      }
  }
  if(dis_front>=stena) gyro_rotate(2);
  else if(dis_left>=stena){
    if(dis_left_b > stena) gyro_rotate(0);
    else{
        while(dis_left_b<stena){
          move(2,50);
          _delay(0.001);
          move(2,0);
          _delay(0.001);
          dis_left_b = IR(A9);
          dis_front_read();
          printl();
          //antizalip();
          okruga();
        }
        move(2,0);
        gyro_rotate(0);
     }
  }
  else if (dis_right>=stena) {
    if(dis_right_b > stena) gyro_rotate(1);
    else{
         while(dis_right_b<stena){
          move(2,50);
          _delay(0.001);
          move(2,0);
          _delay(0.001);
          dis_right_b = IR(A14);
          dis_front_read();
          printl();
          //antizalip();
          okruga();
         }
         move(2,0);
         gyro_rotate(1);
    }
  }
  else{
    gyro_rotate(0);
    gyro_rotate(0);
  }
  Back();   
 // alignment();
 lcd.clear();
  gyro_y();
  _loop();
  }
}
void loop() {
  _loop(); 
}
