#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include <EEPROM.h>

// enables wifi and mqtt - requires secrets.h/cpp

#define WIFI

// Factory settings - can be edited altered in EEPROM menu

#define F_EXPECTED_PACKS 18               //
#define F_CELLS_PER_SLAVE 12              //
#define F_PANIC_MAX_CELL_MILLIVOLTS 4250  // opens contactors
#define F_MAX_CELL_MILLIVOLTS 4200        // shuts down charging
#define F_MIN_CELL_MILLIVOLTS 3300        // opens contactors
#define F_DELTA_CELL_MILLIVOLTS_MAX 100   // max cell imbalance
#define F_BALANCE_MV_THRESHOLD 3700       // pack will stop balancing when cells are under this value
#define F_BALANCE_MV_HYS 10               // target cell delta in mV
#define F_SHUNT_ON_TIME_MS 200            // duty cycle = 1000/ms (keep under 250ms - page 32 PDF)
#define F_SLAVE_KWH 2.73
#define F_MAX_SOC 100       // These values halt charging
#define F_MIN_SOC 5         // and discharging
#define F_MAX_CHARGE 30     // rate in amps - desired W/min pack volts
#define F_MAX_DISCHARGE 30  // rate in amps


// Pack configuration for non-consecutive cells
// #define CELL_CONFIGURATION CELL1 | CELL2 | CELL3 | CELL4 | CELL8 | CELL9 | CELL10 | CELL11 // ZOE 2021 modular = 1-4, 8-11

// GPIO
#define sck 5
#define miso 6
#define mosi 4
#define SS1 7                // chip select for MAX17841
#define SS2 7                // Optional Slave select if Dual UART is used
#define INT_MAX17841_1 8     // Interrupt pin for MAX17841 (unused)
#define SHDNL_MAX17841_1 10  // Shut down pin for MAX17841 (essential)


// CAN IDs for receiver
#define CAB300a 0x3c0
#define CAB300b 0x3c1
#define CAB300c 0x3c2
#define SAMSUNG_SDI 0x10
#define FOX 0x1871
#define LV_INVERTER 0x305

#define PRECHARGE_DWELL_TIME_MS 200  // time between activating precharge and main contactors
#define CAN_TX_INTERSPACE_MS 2       // time between subsequent transmitted CAN frames

// Custom GPIO pins per module type
#if CONFIG_IDF_TARGET_ESP32C6
#define TWAI_TX_SIGNAL_IDX TWAI0_TX_IDX
#define RESET_EEPROM_PIN 9
#define PRECHARGE_PIN 12
#define MAIN_CONTACTOR_PIN 11
#define CAN_TX_PIN 2
#define CAN_RX_PIN 3
#else
#define TWAI_TX_SIGNAL_IDX TWAI_TX_IDX
#define RESET_EEPROM_PIN 3
#define PRECHARGE_PIN 9
#define MAIN_CONTACTOR_PIN 9
#define CAN_TX_PIN 0
#define CAN_RX_PIN 1
#endif


// Timeouts
#define DATA_TIMEOUT_MS 10000  // Time window in which **unique** cell data must be received from the Maxim interface
#define WDT_TIMEOUT_MS 60000   // 1 minute - both WDT use the same watchdog - more is less safety, pick one!
// #define MAXIM_WATCHDOG  // Time window WDT_TIMEOUT_MS in which valid Maxim data must be received or ESP will be reset. Comment out to remove this safety.
// #define INVERTER_WATCHDOG  // Time window WDT_TIMEOUT_MS in which valid inverter CAN frames must be received or ESP will be reset. Comment out to remove this safety.

// ============ Do not edit below here


#define CELL1 (1 << 1)
#define CELL2 (1 << 2)
#define CELL3 (1 << 3)
#define CELL4 (1 << 4)
#define CELL5 (1 << 5)
#define CELL6 (1 << 6)
#define CELL7 (1 << 7)
#define CELL8 (1 << 8)
#define CELL9 (1 << 9)
#define CELL10 (1 << 10)
#define CELL11 (1 << 11)
#define CELL12 (1 << 12)

#define EEPROM_SIZE 512  // Define the size of the EEPROM

struct Configuration {
  uint8_t num_modules;
  uint8_t cells_per_slave;
  uint16_t panic_max_cell_millivolts;
  uint16_t max_cell_millivolts;
  uint16_t min_cell_millivolts;
  uint16_t delta_cell_millivolts_max;
  uint16_t balance_mv_threshold;
  uint8_t balance_mv_hys;
  uint16_t shunt_on_time_ms;
  float slave_kwh;
  uint8_t max_soc;
  uint8_t min_soc;
  uint8_t max_charge;
  uint8_t max_discharge;
  uint8_t pack_voltage_divisor;
};

int max_cells();                // (config.num_modules * config.cells_per_slave)
int max_slave_voltage();        // (config.cells_per_slave * config.panic_max_cell_millivolts / 1000)
int min_slave_voltage();        // (config.cells_per_slave * config.min_cell_millivolts / 1000)
int max_pack_voltage_cutoff();  // (max_slave_voltage() * config.num_modules)  // 108 cells at 100% SoC
int low_pack_voltage_cutoff();  // low_pack_voltage_cutoff() (min_slave_voltage() * config.num_modules)

// Default configuration values
const Configuration defaultConfig = {
  F_EXPECTED_PACKS,             // num_modules (expected)
  F_CELLS_PER_SLAVE,            // config.cells_per_slave
  F_PANIC_MAX_CELL_MILLIVOLTS,  // panic_max_cell_millivolts
  F_MAX_CELL_MILLIVOLTS,        // max_cell_millivolts
  F_MIN_CELL_MILLIVOLTS,        // min_cell_millivolts
  F_DELTA_CELL_MILLIVOLTS_MAX,  // delta_cell_millivolts_max
  F_BALANCE_MV_THRESHOLD,       // balance_mv_threshold
  F_BALANCE_MV_HYS,             // balance_mv_hys
  F_SHUNT_ON_TIME_MS,           // shunt_on_time_ms
  F_SLAVE_KWH,                  // slave_kwh
  F_MAX_SOC,                    // max_soc
  F_MIN_SOC,                    // min_soc
  F_MAX_CHARGE,                 // max_charge
  F_MAX_DISCHARGE,              // max_discharge
  1,                            // pack_voltage_divisor
};


// Function declarations
void loadConfig();
void saveConfig();
void reset_btn();
void resetConfig();
void initializeEEPROM();
void eeprom_menu();
bool changeSetting(char settingNumber);
void displayConfig();
void print_config();

extern Configuration config;
#endif