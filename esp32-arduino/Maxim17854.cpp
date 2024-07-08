#include "Max17852.h"
#include "Maxim.h"

int *Maxim852Device::spi_read(char module, char reg) {
  char command = READALL;
  switch (module) {
    case ALL:
      break;
    default:
      command = module << 3 | READDEVICE;
  }
  char read_num = (READALL == command) ? (5 + (config.cells_per_slave * 2)) : (7);
  // if certain command -> SPI_commands(X, ....) change X and NULL_XX to suit
  PEC_VALUE = pec.pec_code(3, command, reg, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_SCANCTRL,DATA_CHECK
  // bms_SPI.SPI_commands(7, WR_LD_Q0, 6, command, reg, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(7, WR_LD_Q0, read_num, command, reg, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  // Serial.printf("Reader: %x %x %x %x %x %x %x \n\r", WR_LD_Q0, read_num, command, reg, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_read_register(read_num, RD_NXT_MSG);
  // Serial.printf("Read:   %x %x %x %x %x %x %x \n\r", SPI_return[0], SPI_return[1], SPI_return[2], SPI_return[3], SPI_return[4], SPI_return[5], SPI_return[6]);
  // SPI command for Read Load Que
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, 0x00);
  bms_SPI.SPI_commands(2, READ_RX_STATUS, 0x00);
  return SPI_return;
}

int *Maxim852Device::spi_write(char module, char reg, short reg_data) {
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
  PEC_VALUE = pec.pec_code(4, command, reg, lsb, msb);                                        // PEC calculation for BITS WRITEALL, ADDR_MEASUREEN, 0xFF, 0xFF
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x05, command, reg, lsb, msb, PEC_VALUE, ALIVE_COUNTER);  // only 128
  // Serial.printf("Writer: %x %x %x %x %x %x %x %x\n\r", WR_LD_Q0, 0x05, command, reg, lsb, msb, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(6, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);
  bms_SPI.SPI_commands(2, 0x01, NULL_XX);
  return SPI_return;
}


bool Maxim852Device::clear_status_fmea() {
  Serial.println("Init Max17852");
  short reg_val = 0;
  // SPI TRANSACTION : WRITE ADDR_STATUS1 TO 0X00 TO CLEAR FLAGS
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

bool Maxim852Device::enable_measurement() {
  short reg_val = 0;
#ifdef CELL_CONFIGURATION
  reg_val = CELL_CONFIGURATION;
#else
  reg_val = (1 << config.cells_per_slave) - 1;  // 4 cells = 0b000000001111
#endif
  reg_val |= (1 << 14);  // add block volts (total slave voltage) read bit - Page 255 of MAX17854
  spi_write(ALL, ADDR_MEASUREEN1, reg_val);
  delay(1);

  // ADC
  reg_val = 0x3f;  // set auxin adc measurements
  spi_write(ALL, ADDR_MEASUREEN2, reg_val);
  delay(1);

  reg_val = 0;  // set auxin adc measurements
  spi_write(ALL, ADDR_AUXGPIOCFG, reg_val);
  delay(1);

  reg_val = 1;  // set DIAGSEL die adc measurements
  spi_write(ALL, ADDR_DIAGCFG, reg_val);
  delay(1);

  reg_val = 0x1000 | 0x40 | 0x4;  // bal manual seconds | 50% duty | exit on die temp
  spi_write(ALL, ADDR_BALSWCTRL, reg_val);
  delay(1);

  return true;
}

bool Maxim852Device::single_scan() {
  short reg_val = 0x1031;  //
  for (int module = 0; module < config.num_modules; module++) {
    spi_write(module, ADDR_SCANCTRL, reg_val);
  }

  unsigned short raw = 0;
  int counter = 0;
  while (counter <= 20) {
    int success = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
    for (int module = 0; module < config.num_modules; module++) {
      SPI_return = spi_read(module, ADDR_SCANCTRL);
      raw = (SPI_return[2]) + (SPI_return[2 + 1] << 8);
      if ((raw & 0x8000) && (raw & 0x2000)) {
        success++;
      }
    }
    if (success == config.num_modules) { return true; }
    counter++;
  }
  Serial.println("Scan failed");
  return false;
}

void Maxim852Device::read_die_temp() {
  for (int module = 0; module < config.num_modules; module++) {
    SPI_return = spi_read(module, ADDR_DIAG1REG);
    unsigned short raw = (SPI_return[2]) + (SPI_return[2 + 1] << 8);
    float temp = ((raw >> 2) / 16384.0 * 1.25) / 0.00287 - 273 - 4.4;  // Calculation - page 147 MAX17854 PDF
    data->die_temp[module] = (int16_t)temp;
    if ((int16_t)temp > MAX_SLAVE_TEMP) {
      data->errors[module] |= ERROR_TEMP_HIGH;
    }
  }
}
// void Maxim852Device::read_die_temp() {
//   SPI_return = spi_read(ALL, ADDR_DIAG1REG);
//   int module = 0;
//   for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
//     unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);
//     float temp = ((raw >> 2) / 16384.0 * 1.25) / 0.00287 - 273 - 4.4;  // Calculation - page 147 MAX17854 PDF
//     data->die_temp[module] = (int16_t)temp;
//     if ((int16_t)temp > MAX_SLAVE_TEMP) {
//       data->errors[module] |= ERROR_TEMP_HIGH;
//     }
//     module++;
//   }
// }
void Maxim852Device::read_cell_temp() {
  for (int module = 0; module < config.num_modules; module++) {
    SPI_return = spi_read(module, ADDR_AUX2REG);
    unsigned short raw = (SPI_return[2]) + (SPI_return[2 + 1] << 8);
    float temp = calculateTemperature((raw >> 2));
    int temp_16 = (int)temp;  // Calculation of THRM
    data->cell_temp[module] = temp_16;
    if (temp_16 > MAX_SLAVE_TEMP) {
      data->errors[module] |= ERROR_TEMP_HIGH;
    }
    if (temp_16 > data->cell_temp_max) {
      data->cell_temp_max = temp_16;
    }
    if (temp_16 < data->cell_temp_min) {
      data->cell_temp_min = temp_16;
    }
  }
}
// void Maxim852Device::read_cell_temp() {
//   SPI_return = spi_read(ALL, ADDR_AUX1REG);
//   int module = 0;
//   for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
//     unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);
//     float temp = calculateTemperature((raw >> 2));
//     int temp_16 = (int)temp;  // Calculation of THRM
//     data->cell_temp[module] = temp_16;
//     if (temp_16 > MAX_SLAVE_TEMP) {
//       data->errors[module] |= ERROR_TEMP_HIGH;
//     }
//     if (temp_16 > data->cell_temp_max) {
//       data->cell_temp_max = temp_16;
//     }
//     if (temp_16 < data->cell_temp_min) {
//       data->cell_temp_min = temp_16;
//     }
//     module++;
//   }
// }

void Maxim852Device::cell_V() {
  const float CONVERSION = VOLTAGE_REF / VOLTAGE_REF_HEX;
  for (int module = 0; module < config.num_modules; module++) {
    for (char cell_pointer = ADDR_CELL1REG; cell_pointer < (ADDR_CELL1REG + config.cells_per_slave); cell_pointer++) {
      SPI_return = spi_read(module, cell_pointer);
      // for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
      unsigned short raw = ((SPI_return[2]) + (SPI_return[2 + 1] << 8) >> 2);  // local variable
      // important, reset min cell reference for balancing LV ADV val at begining of sampling of first slave
      // +10mv = +33 adc
      if ((0 == module) && (cell_pointer == ADDR_CELL1REG)) {
        min_cell_adc_raw = raw;
        // min_cell_adc_raw = raw + (short)(config.balance_mv_hys * 3.3);
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
        Serial.printf("Error: Module %d (ptr:%d) Cell %d Cell Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.min_cell_millivolts);
        for (int i = 0; i < (5 + (2 * config.cells_per_slave)); i++) {
          Serial.printf(" %x,", SPI_return[i]);
        }
        Serial.println();
        // }
      }
      if (cell_mv > config.panic_max_cell_millivolts) {
        data->errors[module] |= ERROR_CELL_VOLT_HIGH;
        Serial.printf("Error: Module %d Cell %d Cell Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.panic_max_cell_millivolts);
      }
    }
    // }
  }
}
// void Maxim852Device::cell_V() {
//   const float CONVERSION = VOLTAGE_REF / VOLTAGE_REF_HEX;
//   for (char cell_pointer = ADDR_CELL1REG; cell_pointer < ADDR_CELL1REG + config.cells_per_slave; cell_pointer++) {
//     int module = 0;
//     SPI_return = spi_read(ALL, cell_pointer);
//     for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
//       unsigned short raw = ((SPI_return[idx]) + (SPI_return[idx + 1] << 8) >> 2);  // local variable
//       // important, reset min cell reference for balancing LV ADV val at begining of sampling of first slave
//       // +10mv = +33 adc
//       if ((0 == module) && (cell_pointer == ADDR_CELL1REG)) {
//         min_cell_adc_raw = raw + (short)(config.balance_mv_hys * 3.3);
//       } else {
//         if (min_cell_adc_raw > raw) {
//           min_cell_adc_raw = raw;
//         }
//       }
//       float cell_voltage = raw * CONVERSION;
//       // Update min and max cell voltage values
//       short cell_mv = (short)(cell_voltage * 1000.0);
//       char index = cell_pointer - ADDR_CELL1REG + (module * config.cells_per_slave);

//       data->cell_millivolts[index] = cell_mv;
//       if (cell_mv < data->cell_mv_min) {
//         data->cell_mv_min = cell_mv;
//       }
//       if (cell_mv > data->cell_mv_max) {
//         data->cell_mv_max = cell_mv;
//       }
//       if (cell_mv < config.min_cell_millivolts) {
//         data->errors[module] |= ERROR_CELL_VOLT_LOW;
//         Serial.printf("Error: Module %d Cell %d Cell Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.min_cell_millivolts);
//         for (int i = 0; i < (5+(2*config.cells_per_slave)); i++) {
//           Serial.printf(" %x,", SPI_return[i]);
//         }
//         Serial.println();
//         // }
//       }
//       if (cell_mv > config.panic_max_cell_millivolts) {
//         data->errors[module] |= ERROR_CELL_VOLT_HIGH;
//         Serial.printf("Error: Module %d Cell %d Cell Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), int(cell_pointer - ADDR_CELL1REG), cell_mv, config.panic_max_cell_millivolts);
//       }
//       module++;
//     }
//   }
// }

void Maxim852Device::block_V() {
  const float CONVERSION = FULL_SCALE_DCIN / FULL_SCALE_DCIN_HEX;
  for (char module = 0; module < data->num_modules; module++) {
    SPI_return = spi_read(module, ADDR_BLOCKREG);
    unsigned short raw = (SPI_return[2]) + (SPI_return[2 + 1] << 8);  // local variable
    float block_voltage = (raw >> 2) * CONVERSION;
    data->pack_volts += block_voltage;

    if (block_voltage > max_slave_voltage()) {
      Serial.printf("Error: Module %d Block Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, max_slave_voltage());
      data->errors[module] |= ERROR_SLAVE_VOLT_HIGH;
    }
    if (block_voltage < min_slave_voltage()) {
      data->errors[module] |= ERROR_SLAVE_VOLT_LOW;
      Serial.printf("Error: Module %d Block Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, min_slave_voltage());
    }
  }
}


// void Maxim852Device::block_V() {
//   const float CONVERSION = FULL_SCALE_DCIN / FULL_SCALE_DCIN_HEX;
//     Serial.printf("Block v ");
//   SPI_return = spi_read(ALL, ADDR_BLOCKREG);
//   char module = 0;
//   for (char idx = 2; idx < ((data->num_modules * 2) + 2); idx = idx + 2) {
//     unsigned short raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);  // local variable
//     float block_voltage = (raw >> 2) * CONVERSION;
//     data->pack_volts += block_voltage;

//     if (block_voltage > max_slave_voltage()) {
//       Serial.printf("Error: Module %d Voltage High, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, max_slave_voltage());
//       data->errors[module] |= ERROR_SLAVE_VOLT_HIGH;

//     }
//     if (block_voltage < min_slave_voltage()) {
//       data->errors[module] |= ERROR_SLAVE_VOLT_LOW;
//       Serial.printf("Error: Module %d Voltage Low, Measured: %d mV, Limit: %d mV\n\r", int(module), block_voltage, min_slave_voltage());
//     }
//     module++;
//   }
// }

void Maxim852Device::calculate_soc() {
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

void Maxim852Device::calc_balance_bits(char module) {
  // 2. Host determines which cells to balance and associated balancing time.
  data->balance_bits[module] = 0;  // Clear all balance status
  bits_remainder[module] = 0;      // clear all

  for (char cell = 0; cell < config.cells_per_slave; cell++) {
    char index = module * config.cells_per_slave + cell;
    short cell_mv = data->cell_millivolts[index];
    if (cell_balance_conditions(data->cell_mv_min, cell_mv)) {
      // If this cell is within the hysteresis of the lowest cell and above the minimum balance voltage threshold then mark it for balancing.
      data->balance_bits[module] |= 1 << cell;
      data->num_bal_cells++;
    }
  }
}

void Maxim852Device::do_balance() {
for (int balance_counter = 0; balance_counter < 3; balance_counter++) {
    if (balance_counter == 2) {
      SPI_return = spi_read(ALL, ADDR_BALSTAT);
      unsigned short raw = SPI_return[2] | (short)SPI_return[3] << 8;
      char cbtimer = ((raw & 0xF00) >> 8);
      if ((cbtimer < 3) && (data->cell_mv_min > config.balance_mv_threshold)) {
        spi_write(ALL, ADDR_BALEXP1, 60);  // Bal watchdog reset
        // spi_write(ALL, ADDR_BALSWCTRL, 60);  // Bal watchdog reset
      }
      spi_write(ALL, ADDR_BALSWCTRL, 0x8000);  // turn off shunts for voltage measurement and reset wdt
      return;
    }
    unsigned short bits = 0;
    // Counter is cyclical over 1 second and is generated in the main lool (0->3). Each step represents 250ms.
    if (data->cell_mv_max < config.balance_mv_threshold) {  // Whole pack
      // Serial.printf("Balancing off - Cells under threshold %dvM", config.balance_mv_threshold);
      for (char module = 0; module < data->num_modules; module++) {
        data->balance_bits[module] = 0;  // Clear all balance status
        bits_remainder[module] = 0;      // clear all
      }
      spi_write(ALL, ADDR_BALSWCTRL, OFF);
      return;
    }

    for (char module = 0; module < data->num_modules; module++) {
      if (balance_counter == 0) {
        calc_balance_bits(module);
      }
      if (data->die_temp[module] > 100) {  // Check module die overtemp
        Serial.printf("SHUNT OVERTEMP! IC temp: %dºC  Switching off module %d shunts | ", data->die_temp[module], module + 1);
        data->balance_bits[module] = 0;  // Clear all balance status
        bits_remainder[module] = 0;      // clear all
      }
      bits = toggle_adjacent_bits(data->balance_bits[module], &bits_remainder[module]);
      spi_write(module, ADDR_BALSWCTRL, bits);
    }
    vTaskDelay(pdMS_TO_TICKS(config.shunt_on_time_ms));
  }
}
// void Maxim852Device::do_balance() {
//   return;
//   SPI_return = spi_read(ALL, ADDR_BALCTRL);
//   short raw = SPI_return[2] | (short)SPI_return[3] << 8;
//   char cbactive = ((raw & 0xc000) >> 14);

//   switch (cbactive) {
//     case 0:
//     case 2:
//       Serial.println("Cell-Balancing restarting");
//       single_scan();
//       debug_balance();
//       if (cell_balance_conditions(data->cell_mv_min, data->cell_mv_max)) { auto_balance(); }
//       break;
//     case 1:
//       // Serial.println("Cell-Balancing operations are active");
//       read_balance();
//       break;
//     case 3:
//       Serial.println("Cell Balancing Halted Unexpectedly due to Thermal Exit (ALRTCBTEMP), Time Out (ALRTCBTIMEOUT), or Calibration Fault (ALRTCBCAL) Conditions ");
//       debug_balance();
//       break;
//     default:
//       break;
//   }
// }
// void Maxim852Device::read_balance() {
//   data->num_bal_cells = 0;

//   for (int module = 0; module < config.num_modules; module++) {
//     SPI_return = spi_read(module, ADDR_BALSWCTRL);
//     short shunts = SPI_return[2] | (short)SPI_return[2 + 1] << 8;
//     data->balance_bits[module] = shunts;
//     for (int cell = 0; cell < config.cells_per_slave; cell++) {
//       if ((shunts >> cell & 1) == 1) {
//         data->num_bal_cells++;
//       }
//     }
//   }
// }

void Maxim852Device::debug_balance() {
  SPI_return = spi_read(ALL, ADDR_BALCTRL);
  short raw = SPI_return[2] | (short)SPI_return[3] << 8;
  char cbactive = ((raw & 0xc000) >> 14);

  switch (cbactive) {
    case 0:
      Serial.println("Cell Balancing is Disabled ");
      // pi_write(ALL, ADDR_BALAUTOUVTHR, (min_cell_adc_raw < 1))
      break;
    case 1:
      Serial.println("Cell-Balancing Operations are Active");
      break;
    case 2:
      Serial.println("Cell Balancing Completed Normally due to Reaching CBUVTHR or CBEXP Exit Conditions ");
      // auto_balance(result);
      break;
    case 3:
      Serial.println("Cell Balancing Halted Unexpectedly due to Thermal Exit (ALRTCBTEMP), Time Out (ALRTCBTIMEOUT), or Calibration Fault (ALRTCBCAL) Conditions ");
      break;
    default:
      Serial.println("?? 1 ");
      break;
  }
  char cbmode = ((raw & 0x3800) >> 11);
  switch (cbmode) {
    case 0:
      Serial.println("Cell Balancing Disabled (default)");
      break;
    case 1:
      Serial.println("Emergency/EOL Discharge by Hour");
      break;
    case 2:
      Serial.println("Manual Cell Balancing by Second");
      break;
    case 3:
      Serial.println("Manual Cell Balancing by Minute");
      break;
    case 4:
      Serial.println("Auto Individual Cell Balancing by Second");
      break;
    case 5:
      Serial.println("Auto Individual Cell Balancing by Minute");
      break;
    case 6:
      Serial.println("Auto Group Cell Balancing by Second");
      break;
    case 7:
      Serial.println("Auto Group Cell Balancing by Minute");
      break;
    default:
      Serial.println("?? 1 ");
      break;
  }

  char cbduty = ((raw & 0xF0) >> 4);
  Serial.print("Duty cycle ");
  switch (cbduty) {
    case 0:
      printf("6.25%% (default)\n\r");
      break;
    case 1:
      Serial.printf("12.5%%\n\r");
      break;
    case 2:
      Serial.printf("18.75%%\n\r");
      break;
    case 3:
      Serial.printf("25%%\n\r");
      break;
    case 4:
      Serial.printf("31.25%%\n\r");
      break;
    case 5:
      Serial.printf("37.5%%\n\r");
      break;
    case 6:
      Serial.printf("43.75%%\n\r");
      break;
    case 7:
      Serial.printf("50%%\n\r");
      break;
    case 8:
      Serial.printf("56.25%%\n\r");
      break;
    case 9:
      Serial.printf("62.5%%\n\r");
      break;
    case 10:
      Serial.printf("68.75%%\n\r");
      break;
    case 11:
      Serial.printf("75%%\n\r");
      break;
    case 12:
      Serial.printf("81.25%%\n\r");
      break;
    case 13:
      Serial.printf("87.5%%\n\r");
      break;
    case 14:
      Serial.printf("93.75%%\n\r");
      break;
    case 15:
      Serial.printf("100%%, less NOL and measurement/calibration overhead\n\r");
      break;
    default:
      Serial.printf("Unknown\n\r");
      break;
  }
  char cbmeasuren = (raw & 0x3);
  Serial.printf("CBMEASEN %d\n\r", cbmeasuren);

  SPI_return = spi_read(ALL, ADDR_BALSTAT);
  raw = SPI_return[2] | (short)SPI_return[3] << 8;
  Serial.printf("CBTIMER %d \n\r", (raw & 0x3ff));
  Serial.printf("CBCNTR %d \n\r", ((raw & 0xc00) >> 10));
  Serial.printf("CBUNIT %d \n\r", ((raw & 0x3000) >> 12));

  SPI_return = spi_read(ALL, ADDR_BALAUTOUVTHR);
  raw = SPI_return[2] | (short)SPI_return[3] << 8;
  Serial.printf("CBUVTHR %d stored in bms \n\r", ((raw & 0xfffe) >> 2));
  Serial.printf("CBUVTHR %d from cell readings \n\r", min_cell_adc_raw);

  SPI_return = spi_read(ALL, ADDR_STATUS3);
  raw = SPI_return[2] | (short)SPI_return[3] << 8;
  Serial.printf("ADDR_STATUS3 %d  \n\r", raw);
}
void Maxim852Device::auto_balance() {

  // check CBACTIVE = 0b10 and ALRTCBDONE = 0b1
  short cb_mode = (7 << 11);                          // Auto cell balacing by minute
  short cb_duty = ((0xe & 0xf) << 4);                 // 50% duty cycle
  short cb_tempen = (1 << 2);                         // Cell Balancing Halts in Response to ALRTTEMP
  short reg_val = cb_mode | cb_duty | cb_tempen | 3;  // 3 = Embedded ADC/CAL Measurements Enabled, CBUVTHR Checking Enabled

  short bal_hys_adc = (short)((config.balance_mv_hys * 0.001) * VOLTAGE_REF_HEX / VOLTAGE_REF);
  short min_hys_bal_v = ((min_cell_adc_raw + bal_hys_adc) << 2);
  spi_write(ALL, ADDR_BALAUTOUVTHR, min_hys_bal_v);  // write new low cell adc into balance routine
  spi_write(ALL, ADDR_BALEXP1, 5);                   //  5 minutes
  spi_write(ALL, ADDR_BALCTRL, reg_val);
}

inline bool cell_balance_conditions(short min_cell, short cell_mv) {
  return (cell_mv >= config.balance_mv_threshold) && (min_cell < (cell_mv - config.balance_mv_hys));
}

void print_shunts(uint16_t value) {
  for (char i = 0; i < config.cells_per_slave; i++) {
    Serial.printf("%d:%d ", i + 1, (value & (1 << i)) ? 1 : 0);
  }
}

void print_b16(uint16_t value) {
  for (char i = 0; i < 16; i++) {
    Serial.printf("%d", (value & (1 << i)) ? 1 : 0);
  }
  Serial.println();
}

float calculateTemperature(uint16_t value) {
  // Convert r_th to float
  float r_th_f = (float)value;

  // Calculate temperature in Kelvin
  float temp_kelvin = BETA / (BETA / T0 + log(r_th_f / R0));

  // Convert Kelvin to Celsius
  return temp_kelvin - 273.15;
}

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
