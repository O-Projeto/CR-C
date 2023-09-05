#include <Arduino.h>

#define S1 36
#define S2 39
#define S3 34
#define S4 35
#define S5 13
#define S6 4
#define left 18
#define right 19

void setup() {
  Serial.begin(9600);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
}

void loop() {
  int IR2 = analogRead(S2);
  int IR3 = analogRead(S3);
  int IR4 = analogRead(S4);

  Serial.print("S1: ");
  Serial.print(analogRead(S1));
  Serial.print(" | S2: ");
  Serial.print(analogRead(S2));
  Serial.print(" | S3: ");
  Serial.print(analogRead(S3));
  Serial.print(" | S4: ");
  Serial.print(analogRead(S4));
  Serial.print(" | S5: ");
  Serial.println(analogRead(S5));

  if(IR2 > 300){
    Serial.println("VIRA ESQUERDA");
    digitalWrite(left, HIGH);
    digitalWrite(right, LOW);
  }
  if(IR4 > 300){
    Serial.println("VIRA DIREITA");
    digitalWrite(left, LOW);
    digitalWrite(right, HIGH);
  }
  if(IR3 > 1500){
    Serial.println("RETO");
    digitalWrite(left, HIGH);
    digitalWrite(right, HIGH);
  }
}