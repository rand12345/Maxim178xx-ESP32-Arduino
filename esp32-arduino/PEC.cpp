#include "PEC.h"

int poly = 0b10110010;  // PEC/CRC polynomial

int PEC::pec_code(int n, ...) {
  va_list arguments;
  va_start(arguments, n);
  int remainder = 0x00;
  for (int j = 0; j < n; j++) {
    remainder = remainder ^ va_arg(arguments, int);
    for (int i = 0; i < 8; i++) {
      int bitStatus = remainder & 1;  // Just to compare the last bit we are operating and with 0x01
      if (bitStatus == 1) {
        remainder = remainder >> 1;
        remainder = remainder ^ poly;
      } else {
        remainder = remainder >> 1;
      }
    }
  }
  va_end(arguments);
  return remainder;
};
bool PEC::pec_check(int n, int* SPI_return) {

  int remainder = 0x00;
  for (int j = 0; j < n; j++) {
    remainder = remainder ^ SPI_return[j];
    for (int i = 0; i < 8; i++) {
      int bitStatus = remainder & 1;  // Just to compare the last bit we are operating and with 0x01
      if (bitStatus == 1) {
        remainder = remainder >> 1;
        remainder = remainder ^ poly;
      } else {
        remainder = remainder >> 1;
      }
    }
  }
  // return (remainder == SPI_return[n]);
  return true;
};


int pec_uint16(int n, const uint16_t* args) {
  int remainder = 0x00;
  for (int j = 0; j < n; j++) {
    remainder = remainder ^ args[j];
    for (int i = 0; i < 8; i++) {
      int bitStatus = remainder & 1;  // Just to compare the last bit we are operating and with 0x01
      if (bitStatus == 1) {
        remainder = remainder >> 1;
        remainder = remainder ^ poly;
      } else {
        remainder = remainder >> 1;
      }
    }
  }
  return remainder;
}

// legacy - to be removed
bool PEC::PEC_Check(int Start, int End, int return_value[]) {
  int remainder = 0x00;
  for (int q = 0; q <= End; q++) {

    remainder = remainder ^ return_value[q];
    for (int i = 0; i < 8; i++) {
      int bitStatus = remainder & 1;
      if (bitStatus == 1) {
        remainder = remainder >> 1;
        remainder = remainder ^ poly;
      } else {
        remainder = remainder >> 1;
      }
    }
  }
  if (remainder != return_value[6]) {
    return false;
  } else {
    return true;
  }
};
