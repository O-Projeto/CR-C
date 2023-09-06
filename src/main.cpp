#include <Arduino.h>
#include <H_bridge_TB6612.hpp>
#include <config.h>

#define channel1 1
#define resolution_channel1 10
#define channel2 3
#define resolution_channel2 9

Motor left = Motor(AIn1, AIn2, PWMA,  channel1, resolution_channel1);
Motor right = Motor(BIn1, BIn2, PWMB,  channel2, resolution_channel2);

void setup() {
  Serial.begin(9600);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
}

void loop() {
  int IR1 = analogRead(S1);
  int IR2 = analogRead(S2);
  int IR3 = analogRead(S3);
  int IR4 = analogRead(S4);
  int IR5 = analogRead(S5);

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

  if(IR2 > 300){
    Serial.println("VIRA ESQUERDA");
    left.drive(1000);
    right.drive(0);
  }
  if(IR4 > 300){
    Serial.println("VIRA DIREITA");
    right.drive(1000);
    left.drive(1000);
  }
  if(IR3 > 1500){
    Serial.println("RETO");
    left.drive(1000);
    right.drive(1000);
  }
}