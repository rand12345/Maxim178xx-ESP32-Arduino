#include <cstdio>
#include "Initialisation.h"
#include "configuration.h"

void Initialisation::Arduino_SPI_init() {
  Serial.printf("Arduino_SPI_init\n");
  pinMode(SHDNL_MAX17841_1, OUTPUT);     // _SHDN pin on MAX17841 - high is active - low shutsdown ISO-UART bus
  pinMode(SS1, OUTPUT);     // _SHDN pin on MAX17841 - high is active - low shutsdown ISO-UART bus
  digitalWrite(SHDNL_MAX17841_1, LOW);
  digitalWrite(SS1, HIGH);               // disable Slave Select Pin
  delay(100);
  pinMode(SS1, OUTPUT);                  // Setting the Slave Select Pin as Output (custom)
  digitalWrite(SHDNL_MAX17841_1, HIGH);  // Enable MAX17841
  delay(2);
  SPI.begin(sck, miso, mosi, -1);                                         // Initializing the SPI in Micro-controller with CS disabled
  SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));  // Initializing SPI frequency, SPI mode
}

void Initialisation::MAX178X_init_sequence() {
  wakeup();                      // Wake up instructions for start up
  int num_modules = helloall();  // Send Hello all command
  update_num_modules(num_modules); // sets all calcs for min/max pack volts
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
  int check = 0;
  Serial.print("Scanning ISO bus for slaves");
  while (check == 0) {
    delay(50);
    bms_SPI.SPI_commands(5, WR_LD_Q0, 0x03, HELLOALL, NULL_XX, NULL_XX);  // Send the Helloall command to initialize the address of the BMS daisy chain
    bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);                                // Write Next queue
    SPI_return = bms_SPI.SPI_commands(6, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
    bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);  // Read RX_interrupt flags
    bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);
    check = SPI_return[3];
  }
  Serial.println();
  if (check > 32) {
    Serial.printf("Too many slaves detected: %d \nESP32 will reset in 3 seconds...", (int)check);
    delay(3000);
    ESP.restart();
  };
  Serial.printf("Number of slaves in daisy chain is : %d\n\r", (int)SPI_return[3]);
  return check;
}

