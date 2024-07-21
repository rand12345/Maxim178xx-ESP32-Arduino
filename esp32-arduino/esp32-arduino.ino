/*

  Todo:
        PEC validation       ❌
        Read balance shunts  ✅
        Manual balance       ✅
        Auto balance         ❌ (x854)
        Slave temperatures   ✅
        Long reads           ✅ (x823)
        Combine devices      ✅
        WiFi                 ✅
        MQTT                 ✅

*/

#include "esp32-arduino.h"
#include "arduino.h"
#include "driver/twai.h"
#include <freertos/task.h>

void setup() {
  Serial.begin(BAUDRATE);
  initializeEEPROM();
// #ifdef WIFI
  xTaskCreatePinnedToCore(WiFi_init, "WIFI_Task", 8192, NULL, 1, &wifiTaskHandle, 0);
// #endif
  // Contactors off
  digitalWrite(PRECHARGE_PIN, 0);
  digitalWrite(MAIN_CONTACTOR_PIN, 0);
  initialisation.Arduino_SPI_init();

  delay(500);
  print_config();

  initialisation.MAX178X_init_sequence();
  xTaskCreatePinnedToCore(SPI_Task, "SPI_Task", 8192, NULL, 1, &spiTaskHandle, 0);
  Serial.println("Waiting for good battery data");

  for (;;) {
    if (!has_errors(&inverter_data)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  };

  Serial.println("Data for inverter OK - Starting CAN");
  
  rx_queue = xQueueCreate(100, sizeof(twai_message_t));
  tx_queue = xQueueCreate(50, sizeof(twai_message_t));
  xTaskCreatePinnedToCore(TWAI_Task, "TWAI_Task", 4096, NULL, 6, &twaiTaskHandle, 0);
  xTaskCreatePinnedToCore(TWAI_Processing_Task, "TWAI_Processing_Task", 4096, NULL, 4, &twaiprocessTaskHandle, 0);

  vTaskDelay(pdMS_TO_TICKS(5000));
  handle_keypress('?');  // display menu
}

void interrupt_pin() {
  Serial.println("******************** INT LOW ********************");  // placeholder for MAX17841b interrupt handling
}


void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    handle_keypress(incomingByte);
  }
}  // App in threads only


void TWAI_Processing_Task(void *pvParameters) {
  char contactor = 0;
  char status = 0;
  unsigned long summary_time = 0;

#ifdef INVERTER_WATCHDOG
  esp_task_wdt_add(NULL);
#endif

  Serial.println("TWAI Processing Task");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    while (xQueueReceive(rx_queue, (void *)&rxFrame, 10) == pdPASS) {
      vTaskDelay(pdMS_TO_TICKS(1));
      status = can_bus.canread(rxFrame, tx_queue, &inverter_data);
      switch (status) {
        case 0:
        case 1:
          Serial.println("Inv with error");
          Serial.println("Contactors commanded All Off");
          contactor = 0;
          contactor_time = millis();
          digitalWrite(PRECHARGE_PIN, contactor & 1);
          digitalWrite(MAIN_CONTACTOR_PIN, contactor >> 1);

          break;
        case 3:
          switch (contactor) {
            case 0:
              contactor = PRECHARGE;
              contactor_time = millis();
              Serial.println("Contactors commanded Precharge On");
              digitalWrite(PRECHARGE_PIN, contactor & 1);
              digitalWrite(MAIN_CONTACTOR_PIN, contactor >> 1);
              break;
            case 1:
              contactor = PRECHARGE | MAIN;
              contactor_time = millis();
              Serial.println("Contactors commanded Precharge -> Main");
              digitalWrite(MAIN_CONTACTOR_PIN, 1);
              Serial.println("Contactors commanded Main On");
              vTaskDelay(pdMS_TO_TICKS(PRECHARGE_DWELL_TIME_MS));  // overlap
              digitalWrite(PRECHARGE_PIN, 0);
              Serial.println("Contactors commanded Precharge Off");
              contactor = MAIN;

              break;
            default:
              // Set contactor to error state or handle differently
              break;
          }
#ifdef INVERTER_WATCHDOG
          esp_task_wdt_reset();
#endif
          break;
        default:
          break;
      }
    }
    if ((summary_time + 2000) < millis()) {
      can_bus.summary(&inverter_data, print_readings);
      if (print_readings) {
        Serial.printf("Uptime %.1fs\n\r", ((millis() - starttime) * 0.001));
        Serial.printf("Contactors: Precharge: %d | Main: %d\n\n\r", contactor & 1, contactor > 1);
      }
      summary_time = millis();
    }
  }
}

void TWAI_Task(void *pvParameters) {
  // enable panic so ESP32 restarts on can failure
  Serial.println("TWAI_Task Start");
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  int last_can_id = 0;
  driver_installed = true;
  while (1) {
    if (!driver_installed) {
      // Driver not installed
      Serial.println("TWAI driver bad");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
      twai_stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      if (twai_start() == ESP_OK) {
        Serial.println("Driver restarted");
      } else {
        Serial.println("CAN failed to restart driver. Rebooting device.");
        ESP.restart();
      }
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %lu\n\r", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %lu\n\r", twaistatus.rx_overrun_count);
    }
    if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
      ESP_LOGI(EXAMPLE_TAG, "Bus Off state");
      // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
      twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
      for (int i = 3; i > 0; i--) {
        ESP_LOGW(EXAMPLE_TAG, "Initiate bus recovery in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      twai_initiate_recovery();  // Needs 128 occurrences of bus free signal
      ESP_LOGI(EXAMPLE_TAG, "Initiate bus recovery");
      continue;
    }
    if (alerts_triggered & TWAI_ALERT_BUS_RECOVERED) {
      // Bus recovery was successful, exit control task to uninstall driver
      ESP_LOGI(EXAMPLE_TAG, "Bus Recovered");
      // break;
    }

    // Check if message is received and add to threadsafe queue
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      while (twai_receive(&rxFrame, 5) == ESP_OK) {
        if (can_debug) {
          Serial.printf("RX Frame ID: 0x%X, Data Length: %d, Data: ", rxFrame.identifier, rxFrame.data_length_code);
          for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("0x%0X ", rxFrame.data[i]);
          }
          Serial.println();
        }
        // drop repeated can frame is from current measurement device
        if ((last_can_id == rxFrame.identifier) && (rxFrame.identifier == CAB300a || rxFrame.identifier == CAB300b || rxFrame.identifier == CAB300c || rxFrame.identifier == SAMSUNG_SDI)) {
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        last_can_id = rxFrame.identifier;
        if (xQueueSend(rx_queue, (void *)&rxFrame, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("rx_queue overflow");
        }
      }
    }
    // Check if tx messages are queued and transmit on bus
    while (xQueueReceive(tx_queue, (void *)&rxFrame, 5) == pdPASS) {
#ifdef CAN_TX_INTERSPACE_MS
      vTaskDelay(pdMS_TO_TICKS(CAN_TX_INTERSPACE_MS));  // inter-txframe delay
#endif
      vTaskDelay(pdMS_TO_TICKS(1));  // inter-txframe delay
      if (!can_tx_stop) {
        if (can_debug) {
          Serial.printf("TX Frame ID: 0x%X, Data Length: %d, Data: ", rxFrame.identifier, rxFrame.data_length_code);
          for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("0x%0X ", rxFrame.data[i]);
          }
          Serial.println();
        }
        if (twai_transmit(&rxFrame, 20) != ESP_OK) {
          Serial.println("Failed to dequeue tx message for transmission");
        };
      }
    }
  }
  vTaskDelete(NULL);
}



void SPI_Task(void *pvParameters) {
  Maxim *maxim = Maxim::build();
  Serial.println("SPI_Task Maxim instanitated");

#ifdef MAXIM_WATCHDOG
  esp_task_wdt_add(NULL);
#endif
  while (1) {

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis1000 >= interval1000) {
      previousMillis1000 = millis();

      inverter_data = maxim->read_pack();

      if (has_errors(&maxim_data)) {
        Serial.println("SPI_Task Maxim display error");
        maxim->display_error();
      } else {
#ifdef MAXIM_WATCHDOG
        esp_task_wdt_reset();
#endif
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handle_keypress(char incomingByte) {
  if (incomingByte == 'h' || incomingByte == 'H' || incomingByte == '?') {
    // print instructions here
    Serial.println("Instructions: ");
    Serial.println(" - ' ' : Show readings <space>");
    Serial.println(" - 's' : EEPROM settings");
    Serial.println(" - 'c' : Toggle CAN debug");
    Serial.println(" - 'b' : Perform cell debug and balance");
    Serial.println(" - 'm' : Mute/Unmute CAN BUS TX");
    Serial.println(" - 'r' : Kill SPI, TWAI and reboot ESP");
    Serial.println(" - 'h' or '?': Show this help message");
  }

  if (incomingByte == ' ') {
    print_readings = !print_readings;
  }
  if (incomingByte == 's' || incomingByte == 'S') {
    print_readings = false;
    can_debug = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    eeprom_menu();
  }
  if (incomingByte == 'c' || incomingByte == 'C') {
    can_debug = !can_debug;
    Serial.print("CAN debug ");
    Serial.println(can_debug ? "enabled" : "disabled");
  }

  if (incomingByte == 'b' || incomingByte == 'B') {
    cell_debug(&inverter_data);
  }

  if (incomingByte == 'm' || incomingByte == 'M') {
    can_tx_stop = !can_tx_stop;
    Serial.print("CAN Tx ");
    Serial.println(can_tx_stop ? "enabled" : "disabled");
  }

  if (incomingByte == 'r' || incomingByte == 'R') {
    Serial.println("Killing spi...");
    vTaskDelete(spiTaskHandle);
    Serial.println("ISO bus down...");
    Serial.println("Killing twai...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Give some time for the message to be sent
    vTaskDelete(twaiTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));  // Give some time for the message to be sent
    vTaskDelete(twaiprocessTaskHandle);
#ifdef WIFI
    Serial.println("Killing wifi, mqtt...");
    vTaskDelete(wifiTaskHandle);
#endif
    Serial.println("Rebooting the ESP in 1 second...");
    digitalWrite(SHDNL_MAX17841_1, LOW);
    delay(1000);
    ESP.restart();  // Reboot the ESP
  }
}


// #ifdef WIFI
ESP32MQTTClient mqttClient;
void WiFi_init(void *pvParameters) {
  unsigned long mqtt_time = 0;
  Serial.println("WiFi Task");
  wifi_start();

  // for (;;) {
  mqttClient.setURI(MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);

  char mqttClientName[50];
  uint64_t chipid = ESP.getEfuseMac();  // The chip ID is essentially the MAC address
  snprintf(mqttClientName, 50, "MAXIM-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  mqttClient.setMqttClientName(mqttClientName);
  mqttClient.enableLastWillMessage("lwt", "Maxim going offline");
  mqttClient.setKeepAlive(30);
  mqttClient.loopStart();
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(1000));  // low pri-task
    if ((mqtt_time + 5000) > millis()) { continue; }
    mqtt_time = millis();
    char *jsonChar = BMSDataToJson(inverter_data);
    mqttClient.publish(MQTT_TOPIC, jsonChar, 0, false);
    free(jsonChar);
  }
  // }
}

void onMqttConnect(esp_mqtt_client_handle_t client) {
  if (mqttClient.isMyTurn(client))  // can be omitted if only one client
  {
    Serial.println("MQTT connected");
  }
}

void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  mqttClient.onEventCallback(event);
}


// Function to convert BMS_Data to JSON char pointer
char *BMSDataToJson(const BMS_Data &inverter_data) {
  // Create a JSON document
  JsonDocument doc;
  doc["timestamp"] = inverter_data.timestamp;

  JsonArray errors = doc["errors"].to<JsonArray>();
  for (int i = 0; i < inverter_data.num_modules + 1; i++) {
    errors.add(inverter_data.errors[i]);
  }

  doc["num_modules"] = static_cast<int>(inverter_data.num_modules);
  doc["num_bal_cells"] = static_cast<int>(inverter_data.num_bal_cells);
  doc["milliamps"] = inverter_data.milliamps;
  doc["cell_mv_min"] = inverter_data.cell_mv_min;
  doc["cell_mv_max"] = inverter_data.cell_mv_max;
  doc["cell_temp_min"] = inverter_data.cell_temp_min;
  doc["cell_temp_max"] = inverter_data.cell_temp_max;
  doc["pack_volts"] = inverter_data.pack_volts;
  doc["soc"] = inverter_data.soc;

  // Convert JSON document to string
  String jsonString;
  serializeJson(doc, jsonString);

  // Allocate memory for char array and copy the string into it
  char *jsonCharArray = (char *)malloc(jsonString.length() + 1);
  strcpy(jsonCharArray, jsonString.c_str());

  return jsonCharArray;
}

// #endif
