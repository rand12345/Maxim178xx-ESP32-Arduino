#include "Maxim.h"
#include "Max17823.h"



bool Maxim823Device::clear_status_fmea() {
  Serial.println("Init Max17823");
  // SPI TRANSACTION : WRITE ADDR_STATUS1 TO 0X00 TO CLEAR FLAGS
  short reg_val = 0;
  spi_write(ALL, ADDR_STATUS1, reg_val);
  spi_read(ALL, ADDR_STATUS1);
  // SPI TRANSACTION : WRITE ADDR_FMEA1 TO 0X00 TO CLEAR FLAGS
  spi_write(ALL, ADDR_FMEA1, reg_val);
  spi_read(ALL, ADDR_FMEA1);
  // SPI TRANSACTION : WRITE ADDR_FMEA2 TO 0X00 TO CLEAR FLAGS
  spi_write(ALL, ADDR_FMEA2, reg_val);
  spi_read(ALL, ADDR_FMEA2);
  return true;
}

bool Maxim823Device::enable_measurement() {
  Serial.println("SPI_Task Maxim enable_measurement");
  short reg_val = 0;
  spi_write(ALL, ADDR_DEVCFG1, reg_val);
  spi_read(ALL, ADDR_DEVCFG1);

#ifdef CELL_CONFIGURATION
  reg_val = CELL_CONFIGURATION;
#else
  reg_val = (1 << config.cells_per_slave) - 1;    // 4 cells = 0b000000001111
#endif
  reg_val |= (1 << 15);  // Connects the voltage divider to the VBLKP pin. Must be enabled prior to the VBLOCK measurement
  reg_val |= (1 << 14);  // Enables measurement of the VBLKP input in the acquisition mode
  reg_val |= (1 << 13);  // Enables measurement of the AUXIN2 input in the acquisition mode
  reg_val |= (1 << 12);  // Enables measurement of the AUXIN1 input in the acquisition mode
  spi_write(ALL, ADDR_MEASUREEN, reg_val);
  delay(1);
  spi_read(ALL, ADDR_MEASUREEN);

  // reg_val = config.cells_per_slave & 0xf; // xxxx not correct, must be cell_mask's highest 1
  reg_val = top_cell(short(reg_val));
  spi_write(ALL, ADDR_TOPCELL, reg_val);
  delay(1);
  spi_read(ALL, ADDR_TOPCELL);

  reg_val = 0x0006;  // Enable Die temperature measurement
  spi_write(ALL, ADDR_DIAGCFG, reg_val);
  delay(1);
  spi_read(ALL, ADDR_DIAGCFG);

  reg_val = 0x1500;  // Set watchdg for cell balancing to 5s
  spi_write(ALL, ADDR_WATCHDOG, reg_val);
  delay(1);
  spi_read(ALL, ADDR_WATCHDOG);

  delay(1);
  reg_val = 0x305;  // Set therm on
  spi_write(ALL, ADDR_ACQCFG, reg_val);
  spi_read(ALL, ADDR_ACQCFG);
  delay(1);
  return true;
}

bool Maxim823Device::single_scan() {

  spi_write(ALL, ADDR_BALSWEN, OFF);  // turn off shunts for voltage measurement
  vTaskDelay(pdMS_TO_TICKS(1));

  short reg_val = STARTSCAN | X8OVERSAMPLE;  // x8 oversample
  spi_write(ALL, ADDR_SCANCTRL, reg_val);
  int counter = 0;
  while (counter <= 20) {
    vTaskDelay(pdMS_TO_TICKS(5));
    SPI_return = spi_read(ALL, ADDR_SCANCTRL);
    char modules = 0;
    // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
    for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {  // readall
      unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);
      if ((raw & 0x8000) && (raw & 0x2000)) { modules++; };
    }
    if (modules == data->num_modules) { return true; }
    counter++;
  }
  return false;
}

void Maxim823Device::read_die_temp() {
  const float CONV = (230700.0 / 5029581.0);
  SPI_return = spi_read(ALL, ADDR_DIAGREG);
  int module = 0;
  // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
  for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {  // readall
    unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);
    float temp = ((raw >> 2) * CONV) - 273.0;  // Calculation
    data->die_temp[module] = (int16_t)temp;
    module++;
  }
}
void Maxim823Device::read_cell_temp() {
  SPI_return = spi_read(ALL, ADDR_AIN1);
  int module = 0;
  // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
  for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {  // readall
    unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);
    float temp = ((raw >> 4) - 1680) * 0.03125;
    data->cell_temp[module] = (int16_t)temp;
    if (temp > data->cell_temp_max) {
      data->cell_temp_max = temp;
    }
    if (temp < data->cell_temp_min) {
      data->cell_temp_min = temp;
    }
    if ((int16_t)temp > MAX_SLAVE_TEMP) {
      data->errors[module] |= ERROR_TEMP_HIGH;
      Serial.printf("Error: Module %d Temperature High, Measured: %d mV, Limit: %d mV\n\r", int(module), temp, MAX_SLAVE_TEMP);
    }
    module++;
  }
}

void Maxim823Device::cell_V() {
  const float CONVERSION = VOLTAGE_REF / VOLTAGE_REF_HEX;
  short call_mask;
#ifdef CELL_CONFIGURATION
  call_mask = CELL_CONFIGURATION;
#else
  call_mask = (1 << config.cells_per_slave) - 1;  // 4 cells = 0b000000001111
#endif
  short topcell = top_cell(short(call_mask));
  for (char cell_pointer = ADDR_CELL1REG; cell_pointer < ADDR_CELL1REG + topcell; cell_pointer++) {
    if (isIgnoreCell(ADDR_CELL1REG - cell_pointer)) { continue; }
    int module = 0;
    SPI_return = spi_read(ALL, cell_pointer);
    // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
    for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {                 // readall
      unsigned short raw = ((SPI_return[idx]) + (SPI_return[idx + 1] << 8) >> 2);  // local variable
      // important, reset min cell reference for balancing LV ADV val at begining of sampling first slave

      // +10mv = +33 adc
      if ((0 == module) && (cell_pointer == ADDR_CELL1REG)) {
        min_cell_adc_raw = raw + (short)(config.balance_mv_hys * 3.3);
      } else {
        if (min_cell_adc_raw > raw) {
          min_cell_adc_raw = raw;
        }
      }
      float cell_voltage = raw * CONVERSION;
      // Update min and max cell voltage values
      short cell_mv = (short)(cell_voltage * 1000.0);
      char index = cell_pointer - ADDR_CELL1REG + (module * config.cells_per_slave);

      data->cell_millivolts[index] = cell_mv;
      if (cell_mv < data->cell_mv_min) {
        data->cell_mv_min = cell_mv;
      }
      if (cell_mv > data->cell_mv_max) {
        data->cell_mv_max = cell_mv;
      }
      if (cell_mv < config.min_cell_millivolts) {
        data->errors[module] |= ERROR_CELL_VOLT_LOW;
        Serial.printf("Error: Module %d Cell %d Cell Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.min_cell_millivolts);
        for (int i = 0; i < 8; i++) {
          Serial.printf(" %x,", SPI_return[i]);
        }
        Serial.println();
        // }
      }
      if (cell_mv > config.panic_max_cell_millivolts) {
        data->errors[module] |= ERROR_CELL_VOLT_HIGH;
        Serial.printf("Error: Module %d Cell %d Cell Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.panic_max_cell_millivolts);
      }
      module++;
    }
  }
}

void Maxim823Device::block_V() {
  const float CONVERSION = FULL_SCALE_DCIN / FULL_SCALE_DCIN_HEX;
  SPI_return = spi_read(ALL, ADDR_BLOCKREG);
  char module = 0;
  // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
  for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {          // readall
    unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);  // local variable
    float block_voltage = (raw >> 2) * CONVERSION;
    data->pack_volts += block_voltage;

    if (block_voltage > max_slave_voltage()) {
      Serial.printf("Error: Module %d Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, max_slave_voltage());
      data->errors[module] |= ERROR_SLAVE_VOLT_HIGH;
    }
    if (block_voltage < min_slave_voltage()) {
      data->errors[module] |= ERROR_SLAVE_VOLT_LOW;
      Serial.printf("Error: Module %d Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, min_slave_voltage());
    }
    module++;
  }
}

void Maxim823Device::read_balance() {
  SPI_return = spi_read(ALL, ADDR_BALSWEN);
  // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
  for (char idx = (data->num_modules * 2); idx >= 2; idx -= 2) {  // readall
    for (int i = 0; i < data->num_modules; i++) {
      if (i % 3 == 0) {
        Serial.println();
      }
      short shunts = SPI_return[idx] | (short)SPI_return[idx + 1] << 8;
      Serial.printf("# %d -> ", i);
      print_shunts(shunts);
    }
  }
  Serial.println();
}

void Maxim823Device::calc_balance_bits(char module) {
  // Host determines which cells to balance and associated balancing time.
  data->balance_bits[module] = 0;  // Clear all balance status
  bits_remainder[module] = 0;      // clear all

  for (char cell = 0; cell < config.cells_per_slave; cell++) {
    char index = module * config.cells_per_slave + cell;
    short cell_mv = data->cell_millivolts[index];
    if (cell_balance_conditions(data->cell_mv_min, cell_mv)) {
      // If this cell is within the hysteresis of the lowest cell and above the minimum balance voltage threshold then mark it for balancing.
      data->balance_bits[module] |= 1 << cell;
    }
  }
}

void Maxim823Device::do_balance() {
  data->num_bal_cells = 0;
  SPI_return = spi_read(ALL, ADDR_WATCHDOG);
  unsigned short raw = SPI_return[2] | (short)SPI_return[3] << 8;
  char cbtimer = ((raw & 0xF00) >> 8);
  if ((cbtimer < 3) && (data->cell_mv_min > config.balance_mv_threshold)) {
    short reg_val = (10 << 8) | (2 << 12);   // cbtimer = x10 & cbpdiv = 16–240s // you sure about this value?
    spi_write(ALL, ADDR_WATCHDOG, reg_val);  // Bal watchdog reset

    for (char module = 0; module < data->num_modules; module++) {
      calc_balance_bits(module);
    }  // calc new balance shunts
  }


  unsigned short bits = 0;
  // Counter is cyclical over 1 second and is generated in the main lool (0->3). Each step represents 250ms.
  if (data->cell_mv_max < config.balance_mv_threshold) {  // Whole pack
    for (char module = 0; module < data->num_modules; module++) {
      data->balance_bits[module] = 0;  // Clear all balance status
      bits_remainder[module] = 0;      // clear all
    }
    spi_write(ALL, ADDR_BALSWEN, OFF);
    return;
  }

  for (char module = 0; module < data->num_modules; module++) {
    if (data->die_temp[module] > 100) {  // Check module die overtemp
      Serial.printf("SHUNT OVERTEMP! IC temp: %dºC  Switching off module %d shunts | ", data->die_temp[module], module + 1);
      data->balance_bits[module] = 0;  // Clear all balance status
      bits_remainder[module] = 0;      // clear all
    }
    bits = toggle_adjacent_bits(data->balance_bits[module], &bits_remainder[module]);
    spi_write(module, ADDR_BALSWEN, bits);
    data->num_bal_cells += __builtin_popcount(bits);
  }
  vTaskDelay(pdMS_TO_TICKS(config.shunt_on_time_ms));
  //
}

void Maxim823Device::calculate_soc() {
  unsigned int accum = 0;
  char num_cells = data->num_modules * config.cells_per_slave;
  for (char i = 0; i < num_cells; i++) {
    accum += (unsigned int)data->cell_millivolts[i];
  }
  short voltage = (short)(accum / num_cells);

  int voltages[] = { 4200, 4180, 4150, 4100, 4050, 4000, 3920, 3870, 3820, 3780,
                     3750, 3700, 3670, 3630, 3600, 3570, 3530, 3480, 3420, 3350, 3300 };

  float socs[] = { 100, 95, 90, 85, 80, 75, 70, 65, 60, 55,
                   50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0 };
  int numPoints = sizeof(voltages) / sizeof(voltages[0]);

  // Ensure the voltage is within the range
  if (voltage >= 4200) {
    data->soc = 100.0;
    return;
  }
  if (voltage <= 3300) {
    data->soc = 0.0;
    return;
  }

  // Interpolation
  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltages[i] && voltage > voltages[i + 1]) {
      // Linear interpolation
      float slope = (socs[i + 1] - socs[i]) / (voltages[i + 1] - voltages[i]);
      float soc = socs[i] + slope * (voltage - voltages[i]);
      data->soc = soc;
      return;
    }
  }
  // In case voltage is exactly at the last point
  data->soc = socs[numPoints - 1];
  return;
}

void Maxim823Device::debug_balance() {
  SPI_return = spi_read(ALL, ADDR_WATCHDOG);
  short raw = SPI_return[2] | (short)SPI_return[3] << 8;
  char cbdiv = ((raw & 0x7000) >> 12);
  char cbtimer = ((raw & 0xF00) >> 8);
  Serial.printf("CBDIV %x | CBTIMER %x", cbdiv, cbtimer);
}

/*
  Helper functions ===============================================
*/

inline short toggle_adjacent_bits(const short bits, short *bits_remainder) {
  short result = 0;
  short input = bits;        // mutable copy
  input ^= *bits_remainder;  // use *bits_remainder to get the value
  *bits_remainder = 0;       // reset the value pointed to by bits_remainder

  for (int i = 1; i < 16; i++) {
    int first = ((input >> (i - 1)) & 1) == 1;
    int second = ((input >> i) & 1) == 1;

    if (first && second) {
      result |= 1 << (i - 1);           // keep first
      input ^= 1 << i;                  // drop second
      *bits_remainder |= 1 << (i - 1);  // update mask
    } else {
      if (first) {
        result |= 1 << (i - 1);
      }
      if (second) {
        result |= 1 << i;
      }
    }
  }
  return result;
}

void print_shunts(uint16_t value) {
  for (char i = 0; i < config.cells_per_slave; i++) {
    Serial.printf("%d:%d ", i + 1, (value & (1 << i)) ? 1 : 0);
  }
}

short values_crc(const BMS_Data *result) {
  char num_cells = result->num_modules * config.cells_per_slave;
  short crc = pec_uint16(num_cells, result->cell_millivolts);
  return crc;
}

bool timestamp_in_range(const BMS_Data *result, long seconds) {
  return ((result->timestamp + DATA_TIMEOUT_MS) > millis());
}

bool data_ready(const BMS_Data *result) {
  return (0 == result->errors[result->num_modules + 1]);
}

bool has_errors(const BMS_Data *result) {
  for (char i = 0; i < result->num_modules + 1; i++) {
    if (result->errors[i] != 0)
      return true;
  }
  return false;
}

inline bool cell_balance_conditions(short min_cell, short cell_mv) {
  return (cell_mv >= config.balance_mv_threshold) && (min_cell < (cell_mv - config.balance_mv_hys));
}

void print_b16(uint16_t value) {
  for (char i = 0; i < 16; i++) {
    Serial.printf("%d", (value & (1 << i)) ? 1 : 0);
  }
  Serial.println();
}
