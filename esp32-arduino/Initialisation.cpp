#include <cstdio>
#include "Initialisation.h"
#include "configuration.h"

void Initialisation::Arduino_SPI_init() {
  Serial.printf("Arduino_SPI_init\n");
  pinMode(SHDNL_MAX17841_1, OUTPUT);  // _SHDN pin on MAX17841 - high is active - low shutsdown ISO-UART bus
  pinMode(SS1, OUTPUT);               // _SHDN pin on MAX17841 - high is active - low shutsdown ISO-UART bus
  digitalWrite(SHDNL_MAX17841_1, LOW);
  digitalWrite(SS1, HIGH);  // disable Slave Select Pin
  delay(100);
  pinMode(SS1, OUTPUT);                  // Setting the Slave Select Pin as Output (custom)
  digitalWrite(SHDNL_MAX17841_1, HIGH);  // Enable MAX17841
  delay(2);
  SPI.begin(sck, miso, mosi, -1);                                         // Initializing the SPI in Micro-controller with CS disabled
  SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));  // Initializing SPI frequency, SPI mode
}

void Initialisation::MAX178X_init_sequence() {
  wakeup();                         // Wake up instructions for start up
  int num_modules = helloall();     // Send Hello all command
  update_num_modules(num_modules);  // sets all calcs for min/max pack volts
}

void Initialisation::wakeup() {
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_3, KEEP_ALIVE_160_MICRO_SEC);  // Write the configuration 3 register and set keep-alive period to 160us
  bms_SPI.SPI_commands(2, WRITE_RX_INTERRUPT_ENABLE, 0x88);                  // Write Rx_interrupt enable register and set RX_Error_int_enable and RX_Overflow_INT_Enable bits
  bms_SPI.SPI_commands(1, CLR_RX_BUF);                                       // Clear Receive Buffer
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_2, 0x30);                      // WRITE_CONFIGURATION_2 and Enable transmit preambles mode
  bms_SPI.SPI_commands(1, READ_RX_STATUS, NULL_XX);                          // Read RX_Status regiter and check for RX_Status for 0x21, Otherwise repeat transactions
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_2, 0x10);                      // WRITE_CONFIGURATION_2 and Disable transmit preambles mode
  bms_SPI.SPI_commands(1, READ_RX_STATUS);                                   // Read RX_Status register
  bms_SPI.SPI_commands(1, CLR_TX_BUF);                                       // Clear transmit buffer
  bms_SPI.SPI_commands(1, CLR_RX_BUF);                                       // Clear receive buffer
}

int Initialisation::helloall() {
  int module = 0;
  Serial.print("Scanning ISO bus for slaves\n\r");
  int loops = 0;
  while (module == 0 && loops < 100) {
    delay(50);
    loops++;
    bms_SPI.SPI_commands(5, WR_LD_Q0, 0x03, HELLOALL, NULL_XX, NULL_XX);
    bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
    SPI_return = bms_SPI.SPI_commands(6, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
    bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);
    bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);
    module = SPI_return[3];
    if (module > 32) {
      Serial.printf("Too many slaves detected: %d\n\r", module);
      return 0;  // Handle error
    }
  }
  if (module > 0) {
    Serial.printf("Number of slaves in daisy chain is: %d\n\r", module);
    return module;
  }

  Serial.printf("HELLOALL failed - attempting MAX17823b internal loopback check\n\r");
  const char DEVCFG2 = 0x1b;
  const short LASTLOOP_ENABLE = 0x8000;
  const short LASTLOOP_DISABLE = 0x0000;
  const char NUM_MODULES = 16;  // MANUALLY SET WHILST TESTING
  for (char i = 0; i < 32; i++) {
    spi_write(ALL, DEVCFG2, LASTLOOP_DISABLE);
    bms_SPI.SPI_commands(1, CLR_RX_BUF);  // Clear after loopback
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  spi_write((NUM_MODULES - 1), DEVCFG2, LASTLOOP_ENABLE);
  bms_SPI.SPI_commands(2, CLR_RX_BUF);  // Clear after loopback
  vTaskDelay(pdMS_TO_TICKS(10));
  return NUM_MODULES;
  /*
    while (module < 32) {
      vTaskDelay(pdMS_TO_TICKS(10));
      SPI_return = spi_read(module, DEVCFG2);
      short val_data = SPI_return[2] | (short)SPI_return[3] << 8;
      Serial.printf("Module %d %x\n\r", module, val_data);
      Serial.printf("Module %d LOOPBACK=1\n\r", module);
      bms_SPI.SPI_commands(1, CLR_RX_BUF);  // Clear before loopback
      vTaskDelay(pdMS_TO_TICKS(10));
      spi_write(module, DEVCFG2, LASTLOOP_ENABLE);
      vTaskDelay(pdMS_TO_TICKS(10));
      bms_SPI.SPI_commands(1, CLR_RX_BUF);  // Clear after loopback
      vTaskDelay(pdMS_TO_TICKS(10));
      SPI_return = spi_read(module, DEVCFG2);
      val_data = SPI_return[2] | (short)SPI_return[3] << 8;
      Serial.printf("Module %d 1:%x 2:%x\n\r", module, 0x8000, val_data);
      if (0x8000 != (val_data & 0xffff)) {
        Serial.printf("Module %d did not respond - breaking internal loopback check\n\r", module);
        if (module > 0) {
          spi_write(module - 1, DEVCFG2, LASTLOOP_ENABLE);
        }
        break;
      }
      Serial.printf("Module %d LOOPBACK=0\n\r", module);
      spi_write(module, DEVCFG2, LASTLOOP_DISABLE);
      bms_SPI.SPI_commands(1, CLR_RX_BUF);
      module++;
    }
  }
  if (module == 0) {
    Serial.printf("No slaves detected: %d\n\r", module);
    return 0;
  }
  Serial.printf("Number of slaves in daisy chain is: %d\n\r", module);
  return module;
  */
}

int *spi_read(char module, char reg) {
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


  Serial.printf("spi_read C: %x, Reg: %x \n\r", command, reg);
  Serial.printf("spi_read Rx ");
  for (char i = 0; i < 8; i++) {
    Serial.printf("%x, ", SPI_return[i]);
  }
  Serial.printf("\n\r");

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
int *spi_write(char module, char reg, short reg_data) {
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

  Serial.printf("spi_write C: %x, Reg: %x Val: %x \n\r", command, reg, reg_data);
  Serial.printf("\n\r");

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
