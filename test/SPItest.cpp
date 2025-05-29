/*
  ESP32 I2S Microphone Sample
  esp32-i2s-mic-sample.ino
  Sample sound from I2S microphone, display on Serial Plotter
  Requires INMP441 I2S microphone
 
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

#include <Arduino.h>
#include <SPI.h>

#define VSPI_SS 10
#define VSPI_MOSI 11
#define VSPI_MISO 12
#define VSPI_SCLK 13

static SPIClass VSPI = SPIClass(3);

 
void setup() {

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  Serial.begin(115200);

}

void loop(){
  // put your setup code here, to run once:
  byte dataToSend = 0xAA;
  byte receivedData;

  uint8_t command = 0xA5; // Example command
  uint8_t response = 0x00;

  digitalWrite(VSPI_SS, LOW); // Select slave
  delayMicroseconds(10);     // Allow time for slave to get ready

  response = SPI.transfer(command); // Send command and receive response

  digitalWrite(VSPI_SS, HIGH); // Deselect slave

  Serial.print("Sent command: 0x");
  Serial.print(command, HEX);
  Serial.print(" | Received: 0x");
  Serial.println(response, HEX);
  delay(2000);

}