#include <Arduino.h>
#include <H_bridge_TB6612.hpp>
#include <config.h>

#define channel1 1
#define resolution_channel1 10
#define channel2 3
#define resolution_channel2 9

int IR1min = 4095;
int IR2min = 4095;
int IR3min = 4095;
int IR4min = 4095;
int IR5min = 4095;
int IR1max = 0;
int IR2max = 0;
int IR3max = 0;
int IR4max = 0;
int IR5max = 0;
int IR1 = 0;
int IR2 = 0;
int IR3 = 0;
int IR4 = 0;
int IR5 = 0;

Motor left = Motor(AIn1, AIn2, PWMA,  channel1, resolution_channel1);
Motor right = Motor(BIn1, BIn2, PWMB,  channel2, resolution_channel1);

void setup() {
  Serial.begin(9600);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  /*
  while(millis() < 5000){
    IR1 = analogRead(S1);
    IR2 = analogRead(S2);
    IR3 = analogRead(S3);
    IR4 = analogRead(S4);
    IR5 = analogRead(S5);
    
    if(IR1 > IR1max) IR1max = IR1;
    if(IR2 > IR2max) IR2max = IR2;
    if(IR3 > IR3max) IR3max = IR3;
    if(IR4 > IR4max) IR4max = IR4;
    if(IR5 > IR5max) IR5max = IR5;

    if(IR1 < IR1min) IR1min = IR1;
    if(IR2 < IR2min) IR2min = IR2;
    if(IR3 < IR3min) IR3min = IR3;
    if(IR4 < IR4min) IR4min = IR4;
    if(IR5 < IR5min) IR5min = IR5;
  }
  */
}

void loop() {
  IR1 = analogRead(S1);
  IR2 = analogRead(S2);
  IR3 = analogRead(S3);
  IR4 = analogRead(S4);
  IR5 = analogRead(S5);

  Serial.print("S1: ");
  Serial.print(IR1);
  Serial.print(" | S2: ");
  Serial.print(IR2);
  Serial.print(" | S3: ");
  Serial.print(IR3);
  Serial.print(" | S4: ");
  Serial.print(IR4);
  Serial.print(" | S5: ");
  Serial.println(IR5);

/*
  if(IR2 > IR2min/2){
    Serial.println("VIRA ESQUERDA");
    left.drive(1000);
    right.drive(0);
  }
  if(IR4 > IR4min/2){
    Serial.println("VIRA DIREITA");
    right.drive(0);
    left.drive(1000);
  }
  if(IR3 > IR3min/2){
    Serial.println("RETO");
    left.drive(1000);
    right.drive(1000);
  }
*/
  left.drive(1000);
  right.drive(1000);
  delay(10);
}