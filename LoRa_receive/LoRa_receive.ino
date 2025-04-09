/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modified from the examples of the Arduino LoRa library
  More resources: https://RandomNerdTutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/
*********/

#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

// Kalman filter variables
float kalmanGain = 0.0;
float estimateError = 1.0;
float measurementError = 3.0;
float estimatedRSSI = 0.0;

float packetRSSI;
String LoRaData;

// Variables
long sbw = 62500; // test 20800, 62500, 250000
int sf = 7; // test 7, 8, 9, 10, 11, 12

long newSbw;
int newSf;

int updateFlag = 1;

void setup() {
  //initialize Serial Monitor
  Serial.begin(57600);
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //915-938 for Australia
  //863-870 or 433 for Sweden
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  LoRa.setSignalBandwidth(sbw);
  LoRa.setSpreadingFactor(sf); 
  Serial.println("LoRa Initializing OK!");
}

float kalmanFilter(float measurement) {

  // Prediction step
  float prediction = estimatedRSSI;
  estimateError += measurementError;

  // Update step
  kalmanGain = estimateError / (estimateError + measurementError);
  estimatedRSSI = prediction + kalmanGain * (measurement - prediction);
  estimateError = (1 - kalmanGain) * estimateError;

  return estimatedRSSI;
}

void adjustParameters(float distance) {
  if (distance >= 0 && distance < 43.75) {
    sf = 7;
    sbw = 125000;
  } else if (distance >= 43.75 && distance < 56.25) {
    sf = 7;
    sbw = 62500;
  } else if (distance >= 56.25 && distance < 68.75) {
    sf = 8;
    sbw = 125000;
  } else if (distance >= 68.75 && distance < 81.25) {
    sf = 8;
    sbw = 62500;
  } else if (distance >= 81.25 && distance < 93.75) {
    sf = 8;
    sbw = 125000;
  } else if (distance >= 93.75 && distance < 106.25) {
    sf = 9;
    sbw = 62500;
  } else if (distance >= 106.25 && distance < 118.75) {
    sf = 7;
    sbw = 62500;
  } else if (distance >= 118.75) {
    sf = 8;
    sbw = 125000;
  }

  // Only update and send parameters if there is a change or if its the first time
  if (newSf != sf || newSbw != sbw || updateFlag == 1) {
    updateFlag = 0;
    sf = newSf;
    sbw = newSbw;
    LoRa.setSignalBandwidth(sbw);
    LoRa.setSpreadingFactor(sf);
    sendParameters();
  }
}

void sendParameters() {
  LoRa.beginPacket();
  LoRa.print("PARAM ");
  LoRa.print(sf);
  LoRa.print(" ");
  LoRa.print(sbw);
  LoRa.endPacket();
  Serial.println("Sent parameters to sender");
}

void loop() {

  // Try to parse packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {    // received a packet
    while (LoRa.available()) {    // read packet
      LoRaData = LoRa.readString();
      Serial.println("LoRa data: " + LoRaData);
    }

    // Extract velocity from the LoRa data
    /* int spaceIndex = LoRaData.indexOf(' ');
    if (spaceIndex != -1) {
      String velocityStr = LoRaData.substring(0, spaceIndex);
      float velocity = velocityStr.toFloat();

      // Calculate distance (assuming velocity is in m/s and you want distance in meters)
      float distance = velocity * 3; // Modify this calculation based on the estimation

      // Adjust parameters based on distance
      adjustParameters(distance);
    } */

    // RSSI of packet
    packetRSSI = LoRa.packetRssi();
    Serial.printf("RSSI: %.4f\n", packetRSSI);

    // Apply Kalman Filter
    /* estimatedRSSI = kalmanFilter(packetRSSI);
    Serial.printf("Kalman RSSI: %.4f\n", estimatedRSSI); */
  }
}