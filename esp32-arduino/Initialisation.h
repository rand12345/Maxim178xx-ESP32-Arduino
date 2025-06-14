#ifndef INITIALIZATION_H
#define INITIALIZATION_H
#include "Arduino.h"
#include "PEC.h"
#include "BMS_SPI.h"
#include "Max17841.h"
#include "configuration.h"

#define BAUDRATE 115200       // Baud rate for Serial UART communication PC to Microcontroller
#define SPI_FREQUENCY 4000000 // SPI frequency

extern int num_modules;
class Initialisation
{
public:
  void Arduino_SPI_init();
  void MAX178X_init_sequence();

private:
  void wakeup();
  int helloall();
  // int *spi_read(char module, char reg);
  // int *spi_write(char module, char reg, char lowbyte, char highbyte);
  // bool spi_transaction(char reg, char lowbyte, char highbyte);
};
extern Initialisation initialisation;
int *spi_read(char module, char reg);
int *spi_write(char module, char reg, short reg_data);
#endif
