#include <Arduino.h>
#include <H_bridge_TB6612.hpp>
#include <config.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

#define channel1 1
#define channel2 3

QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
BluetoothSerial SerialBT;

Motor left = Motor(AIn1, AIn2, PWMA,  channel1, 10);
Motor right = Motor(BIn1, BIn2, PWMB,  channel2, 10);

double kP = 0;
double kI = 0;
double kD = 0;
double kV = 500;
bool start = false;

void Calib(){
  while(millis() < 10000){
    qtr.calibrate();
    delay(10);
  }

  Serial.println("Calibração Finalizada!");
  SerialBT.println("Calibração Finalizada!");
}

void Iniciar(){
}

void PIDReceive(String ValorRecebido){
    String stgkP = ValorRecebido.substring(2,9);
    kP = stgkP.toDouble();
    String stgkI = ValorRecebido.substring(10,17);
    kI = stgkI.toDouble();
    String stgkD = ValorRecebido.substring(18,25);
    kD = stgkD.toDouble();
    String stgkV = ValorRecebido.substring(26,29);
    kV = stgkV.toDouble();
    kV = kV * 10;

    SerialBT.print(kP, 6);
    SerialBT.print(" | ");
    SerialBT.print(kI, 6);
    SerialBT.print(" | ");
    SerialBT.println(kD, 6);
    SerialBT.print(" | ");
    SerialBT.println(kV, 6);
}

void comunicationBT(){
  if(SerialBT.available()){
    String valorRecebido = SerialBT.readString();
    SerialBT.println(valorRecebido);

    if(valorRecebido == "Cliente Conectado!" || valorRecebido == "Cliente Desconectado!") SerialBT.print(valorRecebido);
    else if(valorRecebido == "a") Calib();
    else if(valorRecebido == "b") start = true;
    else if(valorRecebido == "c") start = false;
    else if(valorRecebido.substring(0, 1) == "d") PIDReceive(valorRecebido);
    
    SerialBT.println(start);
  }
}

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){S1, S2, S3, S4, S5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);

  Serial.begin(9600);
  SerialBT.begin("Cerberus");

  delay(1000);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  //CalibPrint();

  comunicationBT();
  if (start == false){
    left.drive(0);
    right.drive(0);
  }
  else if (start == true){
    left.drive(kV);
    right.drive(kV);
  }

  delay(10);
}