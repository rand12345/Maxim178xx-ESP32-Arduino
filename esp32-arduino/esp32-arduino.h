
#ifndef ESP32_ARDUINO_H
#define ESP32_ARDUINO_H

#include <SPI.h>
#include <stdarg.h>
#include "Maxim.h"
#include "Initialisation.h"
#include "CAN_config.h"
#include <esp_task_wdt.h>
#include "mqtt_client.h"
#include "ESP32MQTTClient.h"

#define EXAMPLE_TAG "TWAI Alert and Recovery"
#define PRECHARGE 1
#define MAIN 2
#define ON 1
twai_message_t rxFrame;

esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Bitmask of all cores
    .trigger_panic = true,
};

int modules = 0;
int PEC_VALUE = 0;
int PEC_check_status = 0;
int *SPI_return = 0;
QueueHandle_t tx_queue, rx_queue;
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t twaiTaskHandle = NULL;
TaskHandle_t twaiprocessTaskHandle = NULL;

BMS_Data maxim_data, inverter_data;
CanBus can_bus;
bool can_debug = false;
bool can_tx_stop = false;
static const unsigned long starttime = millis(); // uptime counter
static unsigned long contactor_time = 0;

unsigned long previousMillis250 = 0;
unsigned long previousMillis1000 = 0;

const long interval250 = 250;   // Minimum balance shunt duration
const long interval1000 = 1000; //

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Look in the api-reference for other speed sets.
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static bool driver_installed = false;

Initialisation initialisation;
PEC pec;
BMS_SPI bms_SPI;

void TWAI_Task(void *pvParameters);
void TWAI_Processing_Task(void *pvParameters);
void SPI_Task(void *pvParameters);
void interrupt_pin();
void handle_keypress(char incomingByte);
void print_config();
#endif
