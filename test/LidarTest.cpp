#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>


#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN 48
#define I2C_RST_PIN 47
#define PWREN_PIN -1

void print_result(VL53L7CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);
void printTable(VL53L7CX_ResultsData *Result);
void rotate_matrix_properly(VL53L7CX_ResultsData *Result, int* proper_matrix);

// Components.
VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_4X4;
char report[256];
int simpleTable[16];

/* Setup ---------------------------------------------------------------------*/
void setup()
{

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L7CX component.
  sensor_vl53l7cx_top.begin();

  sensor_vl53l7cx_top.init_sensor();

  // Start Measurements
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();
}

void loop()
{
  VL53L7CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  uint8_t status;

  do {
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);
    print_result(&Results);
    Serial.println("\n");
  }

  if (Serial.available()>0)
  {
    handle_cmd(Serial.read());
  }
  delay(1000);
}

void rotate_matrix_properly(VL53L7CX_ResultsData *Result, int* proper_matrix){

  int i = 0;
  int j = 0;
  int zones_per_line;
  int number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  for(int y = (zones_per_line - 1); y >= 0; y--){
    for(int x = 0; x < zones_per_line; x++){

      int index = x * zones_per_line + y;
      int proper_index = i * zones_per_line + j;
      int d = Result->distance_mm[index];

      proper_matrix[proper_index] = (int)Result->distance_mm[index];

      j++;
    }

    i++;

  }

  for(i = 0; i < zones_per_line; i++){
    for(j = 0; j < zones_per_line; j++){
        Serial.printf("%5d ", proper_matrix[i * zones_per_line + j]);
    }
    Serial.println();
  }

}

void printSimpleTable(){
  for(int i = 0; i<4; i++){
    for(int j = 0; j<4; j++){
      Serial.print(simpleTable[i*4 +j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void print_result(VL53L7CX_ResultsData *Result){
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  int table[16];

  for (int y = 3, i = 0; y >= 0; y--, i++) {
    for (int x = 0, j = 0; x < 4; x++, j++) {
      int index = x * 4 + y;
      int tableIndex = i * 4 + j;
      int d = Result->distance_mm[index];
      simpleTable[tableIndex] = d;
      Serial.print(simpleTable[tableIndex]);
      Serial.print(" ");
    }
    Serial.println();

  }

  printSimpleTable();

}

void toggle_resolution(void)
{
  sensor_vl53l7cx_top.vl53l7cx_stop_ranging();

  switch (res)
  {
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

void toggle_signal_and_ambient(void)
{
  EnableAmbient = (EnableAmbient) ? false : true;
  EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void)
{
  snprintf(report, sizeof(report),"%c[2J", 27); /* 27 is ESC command */
  SerialPort.print(report);
}

void display_commands_banner(void)
{
  snprintf(report, sizeof(report),"%c[2H", 27); /* 27 is ESC command */
  SerialPort.print(report);

  Serial.print("53L7A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}
