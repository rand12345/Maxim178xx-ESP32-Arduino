#include "BMS_SPI.h"

int return_value[69];
int CS;

int *BMS_SPI::SPI_commands(int num, ...) {

  CS = SS1;  // Connecting chip select of MAX17841 to SS1 of Arduino Mega.
  va_list arguments;
  va_start(arguments, num);
  digitalWrite(CS, LOW);
  for (int x = 0; x < num; x++) {
    return_value[x] = SPI.transfer(va_arg(arguments, int));
  }
  digitalWrite(CS, HIGH);
  va_end(arguments);
  delay(2);
  return return_value;
}
int *BMS_SPI::SPI_read_register(int num, char cmd) {

  CS = SS1;  // Connecting chip select of MAX17841 to SS1 of Arduino Mega.
  digitalWrite(CS, LOW);
  SPI.transfer(cmd);
  for (int x = 0; x < num; x++) {
    return_value[x] = SPI.transfer(cmd);
  }
  digitalWrite(CS, HIGH);
  delay(2);
  return return_value;
}
