/* 

███████╗██╗███╗░░██╗░█████╗░██╗░░░░░
██╔════╝██║████╗░██║██╔══██╗██║░░░░░
█████╗░░██║██╔██╗██║███████║██║░░░░░
██╔══╝░░██║██║╚████║██╔══██║██║░░░░░
██║░░░░░██║██║░╚███║██║░░██║███████╗
╚═╝░░░░░╚═╝╚═╝░░╚══╝╚═╝░░╚═╝╚══════╝

╔═╗╔═╦═╗╔═╦═══╗
║║╚╝║║║╚╝║║╔═╗║
║╔╗╔╗║╔╗╔╗║╚══╗
║║║║║║║║║║╠══╗║
║║║║║║║║║║║╚═╝║
╚╝╚╝╚╩╝╚╝╚╩═══╝
 *  Programme for 2nd Chip 
 * version 2.0
 * 24.09.2022   19.50
 * 
 * encoders
 * Gyro
 */

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
const int Lencoder = 2;
const int Rencoder = 3;
const int buzzer = 9; //buzzer to arduino pin 9

volatile long int Lsteps=0;
volatile long int Rsteps=0;
int distance = 0;
const int circumstance = 12 ;


void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Second chip");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(buzzer, OUTPUT); 
  pinMode(Lencoder,INPUT);
  pinMode(Rencoder,INPUT);
  attachInterrupt(digitalPinToInterrupt(Lencoder),count_L_steps,RISING);
  attachInterrupt(digitalPinToInterrupt(Rencoder),count_R_steps,RISING);
  delay(3000);
  tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(500);   
  noTone(buzzer); 

}

void loop() {
  mpu6050.update();
  distance = (Lsteps*circumstance)/20;
   
  if(Serial.available()){
    if(Serial.read() == 1){
      Serial.write(distance);}
     if(Serial.read() == 0){
      Lsteps = 0;
      Rsteps = 0;}
     if(Serial.read() == 2){
      int angle = mpu6050.getAngleZ();
      Serial.write(angle);}
      }



      
   
    
}


/*
╔═══╦╗─╔╦═╗─╔╦═══╦════╦══╦═══╦═╗─╔╦═══╗
║╔══╣║─║║║╚╗║║╔═╗║╔╗╔╗╠╣╠╣╔═╗║║╚╗║║╔═╗║
║╚══╣║─║║╔╗╚╝║║─╚╩╝║║╚╝║║║║─║║╔╗╚╝║╚══╗
║╔══╣║─║║║╚╗║║║─╔╗─║║──║║║║─║║║╚╗║╠══╗║
║║──║╚═╝║║─║║║╚═╝║─║║─╔╣╠╣╚═╝║║─║║║╚═╝║
╚╝──╚═══╩╝─╚═╩═══╝─╚╝─╚══╩═══╩╝─╚═╩═══╝
*/

void count_L_steps(){
  Lsteps +=1;
}

void count_R_steps(){
  Rsteps +=1;
}
