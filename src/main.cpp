 /**
 ******************************************************************************
 * @file    VL53L7CX_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    16 January 2023
 * @brief   Arduino test application for STMicroelectronics VL53L7CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use these examples you need to connect the VL53L7CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (GND) of the VL53L7CX satellite connected to GND of the Nucleo board
 * pin 2 (IOVDD) of the VL53L7CX satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (AVDD) of the VL53L7CX satellite connected to 5V pin of the Nucleo board
 * pin 4 (PWREN) of the VL53L7CX satellite connected to pin A5 of the Nucleo board
 * pin 5 (LPn) of the VL53L7CX satellite connected to pin A3 of the Nucleo board
 * pin 6 (SCL) of the VL53L7CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (SDA) of the VL53L7CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 8 (I2C_RST) of the VL53L7CX satellite connected to pin A1 of the Nucleo board
 * pin 9 (INT) of the VL53L7CX satellite connected to pin A2 of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <Lidar.h>
#include <SPI.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN 48
#define I2C_RST_PIN 47
#define PWREN_PIN -1

#define VSPI_SS 10
#define VSPI_MOSI 11
#define VSPI_MISO 12
#define VSPI_SCLK 13
#define SPI_INT 14

enum commands
{
    CMD_SET_SPEED = 0x01,    // Command to move motors
    CMD_GET_SPEED = 0x02,    // Command to get current speed
    CMD_TURN_ANGLE = 0x03,   // Command to turn a specific angle
    CMD_GET_DISTANCE = 0x04, // Command to get distance traveled
    CMD_RESET_DISTANCE = 0x05, // Command to reset distance measurement
    CMD_STOP_MOTORS = 0x06,  // Command to stop the motors
};

struct command_set_speed
{
    uint8_t command;     // Command identifier
    int16_t left_speed;  // Speed for the left motor in mm/s
    int16_t right_speed; // Speed for the right motor in mm/s
} __attribute__((packed)) cmd_set_speed;

struct command_get_speed
{
    uint8_t command;     // Command identifier
    int16_t left_speed;  // Speed for the left motor in mm/s
    int16_t right_speed; // Speed for the right motor in mm/s
} __attribute__((packed)) cmd_get_speed;

struct command_turn_angle
{
    uint8_t command; // Command identifier
    int16_t angle;   // Angle to turn in degrees
    int16_t speed;   // Speed for the turn in mm/s
} __attribute__((packed)) cmd_turn_angle;

struct command_get_distance
{
    uint8_t command;        // Command identifier
    int16_t left_distance;  // Distance traveled by the left motor in mm
    int16_t right_distance; // Distance traveled by the right motor in mm
} __attribute__((packed)) cmd_get_distance;

struct command_reset_distance
{
    uint8_t command;        // Command identifier
    int16_t left_distance;  // Reset left distance
    int16_t right_distance; // Reset right distance
} __attribute__((packed)) cmd_reset_distance;

void send_velocity_rpm(float left_wheel, float right_wheel){

  struct command_set_speed data;

  Serial.println(left_wheel);
  Serial.println(right_wheel);

  data.command = CMD_SET_SPEED;
  data.left_speed = (int16_t) left_wheel; 
  data.right_speed = (int16_t) right_wheel;

  Serial.println(data.left_speed);
  Serial.println(data.right_speed);

  uint16_t magic = 0x1234;

  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  SPI.transfer16(magic);

  delayMicroseconds(5);

  SPI.transfer((uint8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  Serial.println("Bonk");

}

int send_get_foward_distance(){

  uint16_t magic = 0x1234;
  struct command_get_distance data;
  int distance_traveled = 0;
  uint8_t response[100] = {0};
  
  commands command = CMD_GET_DISTANCE;

  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  SPI.transfer16(magic);

  delayMicroseconds(5);

  data.command = CMD_GET_DISTANCE;
  data.left_distance = 0;
  data.right_distance = 0;

  SPI.transfer((uint8_t*)&data, sizeof(data));
  delay(10);
  SPI.transfer((uint8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  Serial.println("Bonk");

  distance_traveled = (data.left_distance + data.right_distance)/2;

  Serial.println("Command = " + (String)data.command);
  Serial.println("Left_distance = " + (String)data.left_distance);
  Serial.println("Right_distance = " + (String)data.right_distance);

  Serial.println(distance_traveled);

  return distance_traveled;
}

int front_lidar_distance(){

  int min_value = 5000;

  if(simpleTable[5] != -1)
    min_value = simpleTable[5];
  if(simpleTable[6] != -1)
    min_value = min(simpleTable[5], simpleTable[6]);
  if(simpleTable[7] != -1)
    min_value = min(min_value, simpleTable[7]);
  if(simpleTable[9] != -1)
    min_value = min(min_value, simpleTable[9]);
  if(simpleTable[10] != -1)
    min_value = min(min_value, simpleTable[10]);
  if(simpleTable[11] != -1)
    min_value = min(min_value, simpleTable[11]);

    Serial.print("Min_Value = ");
    Serial.println(min_value);
    
  return min_value;

}

void send_reset_distance(){

  uint16_t magic = 0x1234;
  struct command_reset_distance data;

  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  delayMicroseconds(5);

  SPI.transfer16(magic);

  data.command = CMD_RESET_DISTANCE;
  data.left_distance = 1;
  data.right_distance = 1;

  SPI.transfer((uint8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  Serial.println("Bonk");
}

void send_stop_wheels(){

  uint16_t magic = 0x1234;
  struct command_set_speed data;

  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  delayMicroseconds(5);

  SPI.transfer16(magic);

  data.command = CMD_SET_SPEED;
  data.left_speed = 0;
  data.right_speed = 0;

  SPI.transfer((int8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  Serial.println("Bonk");
}


void avoid_obstacles(float distance, float speed){

  float accumulated_x = 0.0;
  float accumulated_y = 0.0;
  uint8_t stage = 0;

  send_velocity_rpm(0,0);

  delay(5000);

  send_reset_distance();

  delay(3000);

  while(1){

    switch (stage){

    case 0:

      send_velocity_rpm(speed, speed);
      Serial.println("After sending speed inside switch");
      delay(20);

      if(send_get_foward_distance() >= distance){
        delay(20);
        send_velocity_rpm(0, 0);
        Serial.println("Stopped wheels because distance");
        return;
      }

      if(front_lidar_distance() < 50){
        delay(20);
        send_velocity_rpm(0, 0);
        delay(20);
        accumulated_y += send_get_foward_distance();
        Serial.println("Shit");
        //send_turn_angle(90, 20);
        stage = 1;
        return;
      }
  //   case 1:
      
  //     send_velocity_rpm(50, 50);

  //     if(lateral_lidar_distance(left) > 100)
  //       delay(3000);
  //       accumulated_x += send_get_foward_distance();
  //       send_reset_distance();
  //       stage = 2;
  //       break;

  //   case 2:

  //     send_velocity_rpm(50, 50);

  //     if(front_lidar_distance() < 50){
  //       delay(3000);
  //       accumulated_y += send_get_foward_distance();
  //       send_reset_distance();
  //       stage = 1;
  //       break;
  //     }
  // }

  //     if()

    }
    delay(20);
  }
}

void send_turn_angle(float angle, float speed){

  struct command_turn_angle data;

  data.command = CMD_SET_SPEED;
  data.angle = (int16_t) angle; 
  data.speed = (int16_t) speed;

  uint16_t magic = 0x1234;

  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  SPI.transfer16(magic);

  delayMicroseconds(5);

  SPI.transfer((int8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  Serial.println("Bonk");
}

void do_circles(float radius, float time){

  send_velocity_rpm((2.0*3.14*radius - 3.14*100.8)/time, (2.0*3.14*radius + 3.14*100.8)/time);

}

void do_square(float side, float time){

  float action_time = time/8.0;

  for(int i = 0; i < 4; i++){
    send_velocity_rpm(side/action_time, side/action_time);
    delay(50);
    send_velocity_rpm(100.8/(4*action_time), -100.8/(4*action_time));
    delay(50);
  }

}




/* Setup ---------------------------------------------------------------------*/
void setup(){

  // Initialize serial for output
  SerialPort.begin(115200);

  // Initialize I2C bus
  DEV_I2C.begin();

  //SPI Initialization
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);

  pinMode(VSPI_SS, OUTPUT);
  pinMode(SPI_INT, OUTPUT);

  // Configure VL53L7CX component
  sensor_vl53l7cx_top.begin();

  sensor_vl53l7cx_top.init_sensor();

  // Start Measurements
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();

  send_reset_distance();

  delay(5000);

  send_velocity_rpm(0, 0);

  delay(5000);

  send_velocity_rpm(50, 50);

  delay(3000);

  delay(50);
}

void loop(){

  // avoid_obstacles(200, 50);

  uint16_t magic = 0x1234;
  struct command_get_distance data;
  int distance_traveled = 0;
  uint8_t response[100] = {0};
  uint8_t* buffer = (uint8_t*)&data; //[5] = {0x04, 0, 1, 0, 1};

  data.command = CMD_GET_DISTANCE;
  data.left_distance = 0;
  data.right_distance = 0;

  // send_reset_distance();

  SPI.beginTransaction(SPISettings(1e4, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);

  SPI.transfer16(magic);
  SPI.transfer((uint8_t*)&data, sizeof(data));
  delayMicroseconds(50);
  SPI.transfer((uint8_t*)&data, sizeof(data));

  digitalWrite(VSPI_SS, HIGH);

  SPI.endTransaction();

  // SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  // digitalWrite(VSPI_SS, LOW);

  // SPI.transfer(&buffer, 5);
  
  // Serial.println(buffer[0]);

  // digitalWrite(VSPI_SS, HIGH);

  // SPI.endTransaction();

  distance_traveled = (data.left_distance + data.right_distance)/2;

  Serial.println("Command = " + String(data.command));
  Serial.println("Left = " + String(data.left_distance));
  Serial.println("Right = " + String(data.right_distance));

  Serial.println(distance_traveled);

  delay(5000);

}


