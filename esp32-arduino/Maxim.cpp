#include "Maxim.h"

Maxim *Maxim::build() {
  DeviceModel model = detect_device();
  if (model == Maxim852) {
    return new Maxim852Device();
  } else {
    return new Maxim823Device();
  }
}

void Maxim::init_values() {
  data->cell_mv_min = 0xffff;
  data->cell_mv_max = 0;
  data->cell_temp_min = 256.0;
  data->cell_temp_max = -256.0;
  data->pack_volts = 0.0;
  data->num_bal_cells = 0;
}

BMS_Data Maxim::read_pack() {
  init_values();                                     // reset min/max
  data->errors[data->num_modules + 1] = INCOMPLETE;  // cleared on successful read, packs +1 are global errors

  // setup scan parameters
  if (!single_scan()) {
    Serial.println("Error: Maxim scan failed");
    return *data;  // returns data with flagged error "INCOMPLETE"
  }
  for (char i = 0; i < data->num_modules; i++) {
    data->errors[i] = 0;
  }
  cell_V();  // Read cell volts
  validate_cell_delta();
  calculate_soc();

  read_die_temp();   // Read die temp
  read_cell_temp();  // Read cell temp
  block_V();

  do_balance();

  data->pack_volts = data->pack_volts / config.pack_voltage_divisor;
  data->errors[data->num_modules + 1] = 0;
  data->timestamp = millis();
  return *data;
}



void Maxim::print_balance_bits() {
  for (int module = 0; module < data->num_modules; module++) {
    for (char i = 0; i < config.cells_per_slave; i++) {
      Serial.printf("%d:%d ", i + 1, (data->balance_bits[module] & (1 << i)) ? 1 : 0);
    }
  }
}
void Maxim::display_error() {
  for (char i = 0; i < data->num_modules; i++) {
    char error = data->errors[i];
    if (error == 0)
      continue;
    Serial.printf("Module %d ", i);

    if (error & ERROR_CELL_DELTA_HIGH) {
      Serial.printf("Error: Cell Delta High\n\r");
    }
    if (error & ERROR_CELL_VOLT_HIGH) {
      Serial.printf("Error: Cell Voltage High\n\r");
    }
    if (error & ERROR_CELL_VOLT_LOW) {
      Serial.printf("Error: Cell Voltage Low\n\r");
    }
    if (error & ERROR_TEMP_HIGH) {
      Serial.printf("Error: Temperature High\n\r");
    }
    if (error & ERROR_SLAVE_VOLT_HIGH) {
      Serial.printf("Error: Slave Voltage High\n\r");
    }
    if (error & ERROR_SLAVE_VOLT_LOW) {
      Serial.printf("Error: Slave Voltage Low\n\r");
    }
    if (error & PEC_ERROR) {
      Serial.printf("Error: SPI data PEC Error\n\r");
    }
    if (error & INCOMPLETE) {
      Serial.printf("Error: Data read incomplete\n\r");
    }
    if (error & STALE_DATA) {
      Serial.printf("Error: Data is too old\n\r");
    }
  }
}

void cell_debug(const BMS_Data *local_result) {
  char buffer[1024];
  for (char module = 0; module < local_result->num_modules; module++) {
    int len = snprintf(buffer, sizeof(buffer), "Slave: #%d ", module + 1);

    // Add cell information for this module
    for (char cell = 0; cell < config.cells_per_slave; cell++) {
      char index = (module * config.cells_per_slave) + cell;
      len += snprintf(buffer + len, sizeof(buffer) - len,
                      "C%d: %dmV B:%d | ",
                      cell + 1,
                      local_result->cell_millivolts[index],
                      (local_result->balance_bits[module] & (1 << cell)) > 0 ? 1 : 0);

      if (len >= sizeof(buffer)) {
        Serial.print(buffer);
        len = 0;
      }
    }

    // Add temperature information for this module and print the line
    snprintf(buffer + len, sizeof(buffer) - len, "IC Temp: %d \n\r", local_result->die_temp[module]);
    Serial.print(buffer);
  }
}

void Maxim::validate_cell_delta() {
  if ((data->cell_mv_max - data->cell_mv_min) > config.delta_cell_millivolts_max) {
    data->errors[data->num_modules + 1] |= ERROR_CELL_DELTA_HIGH;
  }
}
