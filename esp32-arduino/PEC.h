#ifndef PEC_H
#define PEC_H
#include "Arduino.h"
#include "BMS_SPI.h"
#include <stdarg.h>

extern int PEC_VALUE;
extern int PEC_check_status;

class PEC {
public:
  int pec_code(int n, ...);
  bool pec_check(int n, int* SPI_return);
  bool PEC_Check(int Start, int End, int return_value[]);
};
int pec_uint16(int n, const uint16_t* args);
extern PEC pec;
#endif
