#ifndef MAXIM_H
#define MAXIM_H
#include "Arduino.h"
#include "PEC.h"
#include "bms_SPI.h"
#include "Max17841.h"
#include "configuration.h"

#define MAX_SLAVES 32
#define MAX_SLAVE_TEMP 100  // Maxim die temp
#define ERROR_CELL_DELTA_HIGH 1 << 0
#define ERROR_CELL_VOLT_HIGH 1 << 1
#define ERROR_CELL_VOLT_LOW 1 << 2
#define ERROR_TEMP_HIGH 1 << 3
#define ERROR_SLAVE_VOLT_HIGH 1 << 4
#define ERROR_SLAVE_VOLT_LOW 1 << 5
#define PEC_ERROR 1 << 6
#define INCOMPLETE 1 << 7
#define STALE_DATA 1 << 8

// Forward declaration of derived classes
class Maxim852Device;
class Maxim823Device;

enum DeviceModel {
  Maxim852,
  Maxim823
};

typedef struct
{
  long timestamp;
  int errors[MAX_SLAVES + 1];
  char num_modules;
  char num_bal_cells;
  long milliamps;
  uint16_t cell_mv_min;
  uint16_t cell_mv_max;
  float cell_temp_min;
  float cell_temp_max;
  float pack_volts;
  float soc;
  uint16_t cell_millivolts[MAX_CELLS];  // Array to hold up to 108 cells
  int16_t die_temp[MAX_SLAVES];         // Array to hold MAX17823 IC temperatures
  int16_t cell_temp[MAX_SLAVES];        // Array to hold MAX17823 IC temperatures
  uint16_t balance_bits[MAX_SLAVES];    // Array to hold all cells to be balanced
} BMS_Data;


// ============ Base class ============

class Maxim {

public:
  Maxim() {
    this->data = new BMS_Data();
    this->data->num_modules = config.num_modules;
  }
  virtual ~Maxim() {
    delete this->data;  // Clean up the data pointer
  }

  void display_error();
  void print_balance_bits();
  BMS_Data read_pack();

  static Maxim *build();
protected:
  BMS_Data *data;
  unsigned short min_cell_adc_raw = 0x3fff;  // Automatic balancing algo, lowest of all slaves
  short bits_remainder[32];                  // Manual balancing algo: for keeping track of alternating bit pattern
  char balance_counter = 0;

  void init_values();
  void validate_cell_delta();
  virtual bool clear_status_fmea() = 0;
  virtual bool enable_measurement() = 0;
  virtual bool single_scan() = 0;
  virtual void read_die_temp() = 0;
  virtual void read_cell_temp() = 0;
  virtual void cell_V() = 0;
  virtual void block_V() = 0;
  virtual void calculate_soc() = 0;
  virtual void do_balance() = 0;


  static int *spi_read(char module, char reg) {

    char command = READALL;
    switch (module) {
      case ALL:
        break;
      default:
        command = module << 3 | READDEVICE;
    }
    char read_num = (READALL == command) ? (5 + (config.num_modules * 2) - 1) : (6);
    PEC_VALUE = pec.pec_code(3, command, reg, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_SCANCTRL,DATA_CHECK
    bms_SPI.SPI_commands(7, WR_LD_Q0, read_num, command, reg, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
    bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
    vTaskDelay(pdMS_TO_TICKS(config.num_modules));
    SPI_return = bms_SPI.SPI_read_register(read_num, RD_NXT_MSG);
    // SPI command for Read Load Que
    if (!pec.pec_check(8, SPI_return)) {
      Serial.printf("Rx PEC fail: ");
      for (char i = 0; i < read_num; i++) {
        Serial.printf("%x, ", SPI_return[i]);
      }
      Serial.printf("\n\r");
    }  // Checks the calculated and hardware returned PEC
    bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, 0x00);
    bms_SPI.SPI_commands(2, READ_RX_STATUS, 0x00);

    vTaskDelay(pdMS_TO_TICKS(config.num_modules));
    return SPI_return;
  }
  static int *spi_write(char module, char reg, short reg_data) {
    char command = WRITEALL;
    switch (module) {
      case ALL:
        break;
      default:
        command = module << 3 | WRITEDEVICE;
    }

    char lsb = lowByte(reg_data);
    char msb = highByte(reg_data);
    // if certain command -> SPI_commands(X, ....) change X and NULL_XX to suit
    PEC_VALUE = pec.pec_code(4, command, reg, lsb, msb);                                        // PEC calculation
    bms_SPI.SPI_commands(8, WR_LD_Q0, 0x06, command, reg, lsb, msb, PEC_VALUE, ALIVE_COUNTER);  // only 128
    bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
    vTaskDelay(pdMS_TO_TICKS(1));
    SPI_return = bms_SPI.SPI_read_register(8, RD_NXT_MSG);
    if (!pec.pec_check(8, SPI_return)) {
      Serial.printf("Tx PEC fail: ");
      for (char i = 0; i < 8; i++) {
        Serial.printf("%x, ", SPI_return[i]);
      }
      Serial.printf("\n\r");
    }
    // PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC
    bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);
    bms_SPI.SPI_commands(2, 0x01, NULL_XX);

    vTaskDelay(pdMS_TO_TICKS(config.num_modules));
    return SPI_return;
  }

private:
  // BMS_Data *data;

  static DeviceModel detect_device() {
    Serial.println("Detecting Maxim model");
    SPI_return = spi_read(ALL, 0x0);
    short model = ((SPI_return[2] | short(SPI_return[3] << 8)) >> 4) & 0xfff;
    // Logic to determine the device model based on register 0x0
    Serial.printf("Found model %x\n\r", model);
    if (model == 0x854) {
      return Maxim852;
    } else {
      return Maxim823;
    }
  }
};


// ============ Maxim852Device class ============

class Maxim852Device : public Maxim {
protected:
  unsigned short min_cell_adc_raw = 0x3fff;  // Automatic balancing algo, lowest of all slaves
  char balance_counter = 0;
  static int *spi_read(char module, char reg);
  static int *spi_write(char module, char reg, short reg_data);
public:
  Maxim852Device()
    : Maxim() {
    clear_status_fmea();
    enable_measurement();
  }

private:
  bool clear_status_fmea();
  bool enable_measurement();
  bool single_scan();
  void read_die_temp();
  void read_cell_temp();
  void cell_V();
  void block_V();
  void calculate_soc();
  // void validate_cell_delta();
  void do_balance();
  void read_balance();
  void debug_balance();
  void auto_balance();
  void calc_balance_bits(char module);
};

// ============ Maxim823Device class ============

class Maxim823Device : public Maxim {

public:
  Maxim823Device()
    : Maxim() {
    // this->data = new BMS_Data();
    // this->data->num_modules = config.num_modules;
    clear_status_fmea();
    enable_measurement();
  }
private:
  void init_values();
  bool clear_status_fmea();
  bool enable_measurement();
  bool single_scan();
  void read_die_temp();
  void read_cell_temp();
  void cell_V();
  void block_V();
  void calculate_soc();
  // void validate_cell_delta();
  void do_balance();
  void read_balance();
  void debug_balance();
  void auto_balance();
  void calc_balance_bits(char module);
};

short values_crc(const BMS_Data *result);
bool timestamp_in_range(const BMS_Data *result, long seconds);
bool has_errors(const BMS_Data *result);
bool data_ready(const BMS_Data *result);
inline short toggle_adjacent_bits(const short bits, short *bits_remainder);
inline void print_shunts(unsigned short value);
inline void print_b16(unsigned short value);
bool cell_balance_conditions(short min_cell, short cell_mv);
void cell_debug(const BMS_Data *local_result);

// 852
float calculateTemperature(uint16_t value);

#endif
