#include "configuration.h"

Configuration config;

int max_cells() {
  int val = (config.num_modules * config.cells_per_slave);
  return val;
}
int max_slave_voltage() {
  int val = (config.cells_per_slave * config.panic_max_cell_millivolts / 1000) + 1;
  return val;
}
int min_slave_voltage() {
  int val = (config.cells_per_slave * config.min_cell_millivolts / 1000) - 1;
  return val;
}
int max_pack_voltage_cutoff() {
  int val = (max_slave_voltage() * config.num_modules);  //
  return val;
}
int low_pack_voltage_cutoff() {
  int val = (min_slave_voltage() * config.num_modules);
  return val;
}

void initializeEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();
  reset_btn();
}

void reset_btn() {
  Serial.println("Press and hold MODE button for factory reset menu");
  pinMode(RESET_EEPROM_PIN, INPUT_PULLUP);
  delay(1000);
  if (digitalRead(RESET_EEPROM_PIN) == LOW) {
    Serial.println("\n\nEEPROM CONFIG EDIT - ALL PERIPHERALS OFFLINE UNTIL REBOOT\n\n\r");
    eeprom_menu();
    Serial.println("\n\n\n\nRebooting...\n\n\n\n\r");
    delay(1000);
    ESP.restart();
  }
}

void loadConfig() {
  EEPROM.get(0, config);
  // Check if EEPROM is empty (first byte will be 0xFF if empty)
  if (EEPROM.read(0) == 0xFF) {
    // EEPROM is empty, use default values
    Serial.println("EEPROM corrupt, restoring factory defaults");
    resetConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void resetConfig() {
  config = defaultConfig;
  saveConfig();
}


void eeprom_menu() {
  bool editing = true;
  char settingNumber = Serial.read();
  Serial.println("EEPROM menu: ");
  Serial.println(" - 'd' : Display configurtation and exit.");
  Serial.println(" - 's' : Edit configuration.");
  Serial.println(" - 'r' : Configuration reset to factory defaults and reboot.");
  while (Serial.available() == 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  settingNumber = Serial.read();
  switch (settingNumber) {
    case 'd':
      displayConfig();
      break;
    case 'r':
      resetConfig();
      Serial.println("Configuration reset to factory defaults.");
      ESP.restart();
      break;
    case 's':
      while (editing) {
        displayConfig();
        while (!Serial.available()) {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        settingNumber = Serial.read();
        editing = changeSetting(settingNumber);
      }
      break;
    default:
      Serial.println("Unknown command. Type 'd' to display config, 'r' to reset, 's' for settings.");
  }
  Serial.println("\nExited EEPROM menu\n\r");
}

void displayConfig() {
  Serial.println("\nCurrent Configuration:");
  Serial.printf("1. num_modules: %d (discovered)\n\r", (int)config.num_modules);
  Serial.println("2. cells_per_slave: " + String(config.cells_per_slave));
  Serial.println("3. panic_max_cell_millivolts: " + String(config.panic_max_cell_millivolts));
  Serial.println("4. max_cell_millivolts: " + String(config.max_cell_millivolts));
  Serial.println("5. min_cell_millivolts: " + String(config.min_cell_millivolts));
  Serial.println("6. delta_cell_millivolts_max: " + String(config.delta_cell_millivolts_max));
  Serial.println("7. balance_mv_threshold: " + String(config.balance_mv_threshold));
  Serial.println("8. balance_mv_hys: " + String(config.balance_mv_hys));
  Serial.println("9. shunt_on_time_ms: " + String(config.shunt_on_time_ms));
  Serial.println("a. slave_wh: " + String(int(config.slave_kwh * 1000)));
  Serial.println("b. max_soc: " + String(config.max_soc));
  Serial.println("c. min_soc: " + String(config.min_soc));
  Serial.println("d. max_charge: " + String(config.max_charge));
  Serial.println("e. max_discharge: " + String(config.max_discharge));
  Serial.printf("f. pack_voltage_divisor: %d", (int)config.pack_voltage_divisor);
}

bool changeSetting(char settingNumber) {
  Serial.println("Enter new value followed by CRLF: " + String(settingNumber));
  while (!Serial.available()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  switch (settingNumber) {
    case 0:
      Serial.parseInt();
      Serial.println("Exiting");
      return false;
    case '1':
      Serial.parseInt();
      Serial.println("Number of slaves cannot be changed");
      return false;
      break;
    case '2':
      config.cells_per_slave = Serial.parseInt();
      break;
    case '3':
      config.panic_max_cell_millivolts = Serial.parseInt();
      break;
    case '4':
      config.max_cell_millivolts = Serial.parseInt();
      break;
    case '5':
      config.min_cell_millivolts = Serial.parseInt();
      break;
    case '6':
      config.delta_cell_millivolts_max = Serial.parseInt();
      break;
    case '7':
      config.balance_mv_threshold = Serial.parseInt();
      break;
    case '8':
      config.balance_mv_hys = Serial.parseInt();
      break;
    case '9':
      config.shunt_on_time_ms = Serial.parseInt();
      break;
    case 'a':
      config.slave_kwh = float(Serial.parseInt() * 1000);
      break;
    case 'b':
      config.max_soc = Serial.parseInt();
      break;
    case 'c':
      config.min_soc = Serial.parseInt();
      break;
    case 'd':
      config.max_charge = Serial.parseInt();
      break;
    case 'e':
      config.max_discharge = Serial.parseInt();
      break;
    case 'f':
      config.pack_voltage_divisor = Serial.parseInt();
      break;
    default:
      Serial.parseInt();
      Serial.println(" Invalid setting number, exiting.");
      return false;
  }
  saveConfig();
  Serial.println("\nEEPROM updated.");
  return true;
}

void print_config() {
  Serial.printf("\nSystem Settings:\n\r");
  Serial.printf("config.num_modules: %d\n\r", config.num_modules);
  Serial.printf("config.cells_per_slave: %d\n\r", config.cells_per_slave);
  Serial.printf("config.shunt_on_time_ms: %d ms\n\r", config.shunt_on_time_ms);
  Serial.printf("config.panic_max_cell_millivolts: %d mV (opens contactors)\n\r", config.panic_max_cell_millivolts);
  Serial.printf("config.max_cell_millivolts: %d mV (shuts down charging)\n\r", config.max_cell_millivolts);
  Serial.printf("config.min_cell_millivolts: %d mV (opens contactors)\n\r", config.min_cell_millivolts);
  Serial.printf("config.delta_cell_millivolts_max: %d mV (max cell imbalance)\n\r", config.delta_cell_millivolts_max);
  Serial.printf("max_slave_voltage: %d V\n\r", max_slave_voltage());
  Serial.printf("min_slave_voltage: %d V\n\r", min_slave_voltage());
  Serial.printf("config.balance_mv_threshold: %d mV (pack will stop balancing when cells are under this value)\n\r", config.balance_mv_threshold);
  Serial.printf("config.balance_mv_hys: %d mV (target cell delta)\n\r", config.balance_mv_hys);
  Serial.printf("config.slave_kwh: %.2f kWh\n\r", config.slave_kwh);
  Serial.printf("config.max_soc: %d %% (halt charging)\n\r", config.max_soc);
  Serial.printf("config.min_soc: %d %% (halt discharging)\n\r", config.min_soc);
  Serial.printf("config.max_charge: %d A (rate in amps - desired W/min pack volts)\n\r", config.max_charge);
  Serial.printf("config.max_discharge: %d A (rate in amps)\n\r", config.max_discharge);
  Serial.printf("max_pack_voltage_cutoff: %d V \n\r", max_pack_voltage_cutoff());
  Serial.printf("low_pack_voltage_cutoff: %d V \n\r", low_pack_voltage_cutoff());
  Serial.printf("DATA_TIMEOUT_MS: %d ms (unique cell data timeout from the Maxim interface)\n\r", DATA_TIMEOUT_MS);
  Serial.printf("WDT_TIMEOUT_MS: %d ms (1 minute)\n\r", WDT_TIMEOUT_MS);
#ifdef MAXIM_WATCHDOG
  Serial.printf("MAXIM_WATCHDOG: enabled. Valid Maxim data must be received or ESP will be reset based on WDT_TIMEOUT_MS. \n\r");
#endif
#ifdef INVERTER_WATCHDOG
  Serial.printf("INVERTER_WATCHDOG: enabled. Valid inverter CAN frames must be received or ESP will be reset based on WDT_TIMEOUT_MS. \n\r");
#endif
  Serial.println();
}