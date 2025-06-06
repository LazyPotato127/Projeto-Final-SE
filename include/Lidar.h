#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_4X4;
int simpleTable[16];

#define LPN_PIN 48
#define I2C_RST_PIN 47
#define PWREN_PIN -1

VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

void get_matrix(VL53L7CX_ResultsData *Result);
void print_simple_table();
void toggle_resolution();
int check_distance_less_than(int dist);

void print_simple_table(){
  for(int i = 0; i<4; i++){
    for(int j = 0; j<4; j++){
      Serial.printf("%5d", simpleTable[i * 4 + j]);
    }
    Serial.println();
  }
}


void get_matrix(VL53L7CX_ResultsData *Result){
    
    uint8_t zones_per_line;
    uint8_t number_of_zones = res;

    zones_per_line = (number_of_zones == 16) ? 4 : 8;

    int i = 0;
    int j = 0;

    for (int y = (zones_per_line - 1), i = 0; y >= 0; y--, i++) {
        for (int x = 0, j = 0; x < zones_per_line; x++, j++) {

        int index = x * 4 + y;
        int tableIndex = i * 4 + j;

        if(Result->nb_target_detected[index]>0){
            int d = Result->distance_mm[index];
            simpleTable[tableIndex] = d;
        }

        else{
            simpleTable[tableIndex] = -1;
        }
        }

        Serial.println();

    }

    // print_simple_table();

}


int check_distance_less_than(int dist){

    int zones_per_line = (res == 16) ? 4 : 8;
    
    for(int i = 0; i < zones_per_line; i++){
        for(int j = 0; j < zones_per_line; j++){

            int d = simpleTable[i * zones_per_line + j];

            if(d != -1 && d < dist){
                return 1;
            }

        }
    }

    return 0;

}


void toggle_resolution(){
  sensor_vl53l7cx_top.vl53l7cx_stop_ranging();

  switch (res){
    case VL53L7CX_RESOLUTION_4X4:
      res = VL53L7CX_RESOLUTION_8X8;
      break;

    case VL53L7CX_RESOLUTION_8X8:
      res = VL53L7CX_RESOLUTION_4X4;
      break;

    default:
      break;
  }
  sensor_vl53l7cx_top.vl53l7cx_set_resolution(res);
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();
}


