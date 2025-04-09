/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modified from the examples of the Arduino LoRa library
  More resources: https://RandomNerdTutorials.com/
esp32-lora-rfm95-transceiver-arduino-ide/
*********/

#include <LoRa.h>
#include <ESP32-TWAI-CAN.hpp>
#include <Arduino.h>
#include "SD_card.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Define the pins used by the transceiver module
#define ss 10
#define rst 9
#define dio0 8

// Define pins for CAN tranceiver
#define CAN_TX		5
#define CAN_RX		4

// Define pins for miso mosi and clock
#define sck 13
#define miso 12
#define mosi 11

// Define chip select for SD card
#define sd_cs 14

SPIClass spiLoRa(FSPI);

// Define variables for CAN-data transmission
float battery_volt;
float battery_current;
float battery_cell_LOW_volt;
float battery_cell_HIGH_volt;
float battery_cell_AVG_volt;
float battery_cell_LOW_temp;
float battery_cell_HIGH_temp;
float battery_cell_AVG_temp;
float battery_cell_ID_HIGH_temp;
float battery_cell_ID_LOW_temp;
float BMS_temp;

float velocity;
float distance_travelled;
float motor_current;
float motor_temp;
float motor_controller_temp;
float driverRPM;
float driver_current;

float MPPT1_watt;
float MPPT2_watt;
float MPPT3_watt;
float MPPT_total_watt;

int left_blinker;
int right_blinker;
int hazard_light;
int brake_light;
int horn;

CanFrame msg;


// Flag for writing in same folder on the SD-card
int SDcardFlag = 1;

// Variables for blinking LEDs
int count = 0;
unsigned long previousMillisLEDs = 0;
const int blinkInterval = 500;
bool ledState = LOW;


/** LoRa parameters **/

// Transmitting frequency
float sendInterval = 1.0;

// Spreading factor (test 7-12)
int sf = 7;

// Signal bandwidth (test 20.8e3, 62.5e3, 125e3, and 250e3)
long sbw = 125000;

void assignCAN2variable() {

  // For MPPT power out
  double voltOut = ((double)(msg.data[4] * 256 + msg.data[5]))/100;
  double currentOut;
    if (msg.data[6] >= 128)
      currentOut = ((double)((msg.data[6] - 256) * 256 + msg.data[7]))*0.0005;
    else
      currentOut = ((double)(msg.data[6] * 256 + msg.data[7]))*0.0005;
  
  switch(msg.identifier) {
    case 0x402:
      motor_current = *((float*)(msg.data+4));
      break;
    case 0x403:
      velocity = *((float*)(msg.data+4));
      break;
    case 0x40B:
      motor_controller_temp = *((float*)(msg.data+4));
      motor_temp = *((float*)(msg.data));
      break;
    case 0x40E:
      distance_travelled = *((float*)(msg.data));
      break;

    // Battery temperatures
    case 0x601:
      BMS_temp = msg.data[1];
      battery_cell_HIGH_temp = msg.data[2];
      battery_cell_LOW_temp = msg.data[3];
      battery_cell_AVG_temp = msg.data[4];
      battery_cell_ID_HIGH_temp = msg.data[5];
      battery_cell_ID_LOW_temp = msg.data[6];
      break;

    // Battery voltage and current
    case 0x602:
      if(msg.data[0] >= 128)
        battery_current = ((double)(msg.data[0] - 256) * 256 + msg.data[1])/10;
      else
        battery_current = ((double)(msg.data[0] * 256 + msg.data[1]))/10;
      battery_volt = ((double)(msg.data[2] * 256 + msg.data[3]))/10;
      battery_cell_LOW_volt = ((double)(msg.data[4] * msg.data[5]))/10000;
      battery_cell_HIGH_volt = ((double)(msg.data[6] * msg.data[7]))/10000;
      break;

    case 0x603:
      battery_cell_AVG_volt = ((double)(msg.data[0] * 256 + msg.data[1]))/10000;
      break;
      
    //MPPT power out (watt)
    case 0x200:
      MPPT1_watt = voltOut * currentOut;
      break;
    case 0x210:
      MPPT2_watt = voltOut * currentOut;
      break;
    case 0x220:
      MPPT3_watt = voltOut * currentOut;
      break;
    
    //Blinkers, hazards, and brake
    case 0x176:
      left_blinker = msg.data[0];
      right_blinker = msg.data[1];
      hazard_light = msg.data[2];
      brake_light = msg.data[3];
      Serial.printf("left: %d, right %d, hazard: %d, brake: %d", left_blinker, right_blinker, hazard_light, brake_light);
      Serial.println();

    case 0x501:
      driver_current = *((float*)(msg.data+4));
      driverRPM = *((float*)(msg.data));
      //Serial.printf("Driver current: %.2f, Driver RPM: %.2f", driver_current, driverRPM);
      //Serial.println();
      
    default:
      //Serial.println("No matching identifier");
      break;
  }
  MPPT_total_watt = MPPT1_watt + MPPT2_watt + MPPT3_watt;
}

void prepareAndWrite2SD() {
  char buffer[512];
  snprintf(buffer, sizeof(buffer),
            "%.2f %.2f "
            "%.2f %.2f "
            "%.2f %.2f %.2f "
            "%.2f %.2f %.2f "
            "%.2f %.2f %.2f "
            "%.2f %.2f %.2f "
            "%.2f %.2f %.2f %.2f \n",
            velocity, distance_travelled, 
            battery_volt, battery_current, 
            battery_cell_LOW_volt, battery_cell_HIGH_volt, battery_cell_AVG_volt,
            battery_cell_LOW_temp, battery_cell_HIGH_temp, battery_cell_AVG_temp,
            battery_cell_ID_HIGH_temp, battery_cell_ID_LOW_temp, BMS_temp,
            motor_current, motor_temp, motor_controller_temp,
            MPPT1_watt, MPPT2_watt, MPPT3_watt, MPPT_total_watt);
  write2SDcard(buffer, SDcardFlag);
}

void sendLoRaData() {

    while(LoRa.beginPacket() == 0) {
      Serial.println("Waiting for radio...");
      delay(100);
    }

    LoRa.beginPacket();
    
    LoRa.print(velocity); LoRa.print(" ");
    LoRa.print(distance_travelled); LoRa.print(" ");
    LoRa.print(battery_volt); LoRa.print(" ");
    LoRa.print(battery_current); LoRa.print(" ");
    LoRa.print(battery_cell_LOW_volt); LoRa.print(" ");
    LoRa.print(battery_cell_HIGH_volt); LoRa.print(" ");
    LoRa.print(battery_cell_AVG_volt); LoRa.print(" ");
    LoRa.print(battery_cell_LOW_temp); LoRa.print(" ");
    LoRa.print(battery_cell_HIGH_temp); LoRa.print(" ");
    LoRa.print(battery_cell_AVG_temp); LoRa.print(" ");
    LoRa.print(battery_cell_ID_HIGH_temp); LoRa.print(" ");
    LoRa.print(battery_cell_ID_LOW_temp); LoRa.print(" ");
    LoRa.print(BMS_temp); LoRa.print(" ");
    LoRa.print(motor_current); LoRa.print(" ");
    LoRa.print(motor_temp); LoRa.print(" ");
    LoRa.print(motor_controller_temp); LoRa.print(" ");
    LoRa.print(MPPT1_watt); LoRa.print(" ");
    LoRa.print(MPPT2_watt); LoRa.print(" ");
    LoRa.print(MPPT3_watt); LoRa.print(" ");
    LoRa.print(MPPT_total_watt);

    LoRa.endPacket();
}

void blinkLEDs() {

  unsigned long currentMillis = millis();

  if(currentMillis - previousMillisLEDs >= blinkInterval) {
    previousMillisLEDs = currentMillis;
    ledState = !ledState;

    // Blink LEDS
    if (left_blinker > 0.0 || hazard_light > 0.0) {
    digitalWrite(15, ledState);
    }
    else
      digitalWrite(15, LOW);
    
    if (right_blinker > 0.0 || hazard_light > 0.0) {
      digitalWrite(16, ledState);
    }
    else
      digitalWrite(16, LOW);
    
    if (brake_light > 0.0) {
      digitalWrite(7, HIGH);
    }
    else
      digitalWrite(7, LOW);
  }
}

void setup() {

  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("LoRa Sender");

  // begin custom SPI
  spiLoRa.begin(sck, miso, mosi, ss);

  //setup LoRa transceiver module
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(ss, rst, dio0);

  // Set pins
	ESP32Can.setPins(CAN_TX, CAN_RX);
	
  // You can set custom size for the queues - these are default
  ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  if(ESP32Can.begin()) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }

  if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 5, 5)) {
    Serial.println("CAN bus speed 500!");
  } else {
    Serial.println("CAN bus failed!");
  }

  //915-938 for Australia
  //863-870 or 433 for Sweden
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }

  // Set spreading factor
  LoRa.setSpreadingFactor(sf);
  // Set bandwidth
  LoRa.setSignalBandwidth(sbw);

  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  // Initialize SD
  if(!SD.begin(sd_cs, spiLoRa)) {
    Serial.println("SD Card Mount Failed");
  } else {
    Serial.println("SD Card Initialized");
  }

  // Blinkers setup
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void loop() {

  static clock_t lastSendTime = 0;

  if (ESP32Can.readFrame(msg, 0)) {
    Serial.printf("Received CAN ID: %3X \r\n", msg.identifier);

    // Assign CAN data to specified variable based on identifier
    assignCAN2variable();

    //Blink LEDs
    blinkLEDs();

    clock_t currentTime = clock();
    double timeElapsed = ((double)(currentTime - lastSendTime))/CLOCKS_PER_SEC;

    if (timeElapsed >= sendInterval) {

      /* //Print information form driver
      if(driver_current > 0.0 || driverRPM > 0.0) {
        Serial.printf("Driver current: %.2f, Driver RPM: %.2f", driver_current, driverRPM);
        Serial.println();
      }
      if(left_blinker > 0 || right_blinker > 0 || hazard_light > 0 || brake_light > 0) {
        Serial.printf("left: %d, right %d, hazard: %d, brake: %d", left_blinker, right_blinker, hazard_light, brake_light);
        Serial.println();
      } */

      // Prepare const char of our variables and save to SD card
      prepareAndWrite2SD();
      digitalWrite(ss, LOW);
      digitalWrite(sd_cs, HIGH);
      Serial.println("Wrote to SD");
      digitalWrite(ss, HIGH);
      digitalWrite(sd_cs, LOW);

      // Send with LoRa
      sendLoRaData();
      Serial.println("CAN sent with LoRa");
      lastSendTime = currentTime;
    }
  }

  // Check for incoming parameters from receiver
  /* int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedParams = "";
    while (LoRa.available()) {
      receivedParams += (char)LoRa.read();
    }
    if (receivedParams.startsWith("PARAM")) {
      int sfIndex = receivedParams.indexOf(' ', 6);
      int sbwIndex = receivedParams.indexOf(' ', sfIndex + 1);
      sf = receivedParams.substring(6, sfIndex).toInt();
      sbw = receivedParams.substring(sfIndex + 1, sbwIndex).toInt();

      LoRa.setSpreadingFactor(sf);
      LoRa.setSignalBandwidth(sbw);

      Serial.printf("Updated SF: %d, SBW: %ld\n", sf, sbw);
    }
  } */
}