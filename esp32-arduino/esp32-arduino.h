#ifndef ESP32_ARDUINO_H
#define ESP32_ARDUINO_H

#ifdef CONFIG_IDF_TARGET_ESP32C3
#include "M5StampC3LED.h"
M5StampC3LED led = M5StampC3LED();
#endif

#include <SPI.h>
#include <stdarg.h>
#include "Maxim.h"
#include "Initialisation.h"
#include "CAN_config.h"
#include <esp_task_wdt.h>
#include "ESP32MQTTClient.h"
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager

#include <ESPmDNS.h> // OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiManager wm;
MQTTConfig mqtt_config;

WiFiManagerParameter custom_mqtt_server("server", "MQTT server", "", 40);
WiFiManagerParameter custom_mqtt_user("user", "MQTT user", "", 40);
WiFiManagerParameter custom_mqtt_password("password", "MQTT password", "", 40);
WiFiManagerParameter custom_mqtt_topic("topic", "MQTT topic", "", 80);
WiFiManagerParameter custom_num_slaves("18", "Number of slaves", "", 2);
WiFiManagerParameter custom_num_cells("6", "Number of cells", "", 2);
WiFiManagerParameter custom_max_soc("100", "Max SoC", "", 3);
WiFiManagerParameter custom_min_soc("10", "Min SoC", "", 2);

char espClientName[50];
bool ap_mode = false;

TaskHandle_t wifiTaskHandle = NULL;

#define EXAMPLE_TAG "TWAI Alert and Recovery"
#define PRECHARGE 1
#define MAIN 2
#define ON 1
twai_message_t rxFrame;

int modules = 0;
int PEC_VALUE = 0;
int PEC_check_status = 0;
int *SPI_return = 0;
QueueHandle_t tx_queue, rx_queue;
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t twaiTaskHandle = NULL;
TaskHandle_t twaiprocessTaskHandle = NULL;

CanBus can_bus;
bool can_debug = false;
bool can_tx_stop = false;
static const unsigned long starttime = millis();  // uptime counter
static unsigned long contactor_time = 0;

unsigned long previousMillis1000 = 0;
const long interval1000 = 1000;  

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // Look in the api-reference for other speed sets.
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static bool driver_installed = false;

Initialisation initialisation;
PEC pec;
BMS_SPI bms_SPI;

void MQTT_task(void *pvParameters);
char *BMSDataToJson(const BMS_Data &data);
void TWAI_Task(void *pvParameters);
void TWAI_Processing_Task(void *pvParameters);
void SPI_Task(void *pvParameters);
void interrupt_pin();
void handle_keypress(char incomingByte);
void print_config();
#endif
