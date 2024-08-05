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
  pinMode(RESET_EEPROM_PIN, INPUT_PULLUP);
  // Contactors off
  digitalWrite(PRECHARGE_PIN, 0);
  digitalWrite(MAIN_CONTACTOR_PIN, 0);

  Serial.begin(BAUDRATE);
#ifdef RGBLED
  led.clear();
#endif

  initializeEEPROM();
#ifdef RGBLED
  led.red(255).update();
#endif

  if (setup_wifi()) {
    ota();
    xTaskCreatePinnedToCore(STA_task, "STA_task", 8192, NULL, 1, &wifiTaskHandle, 0);
  } else {
    ap_mode = true;
  }
#ifdef RGBLED
  if (ap_mode) {
    led.blue(255).update();
  } else {
    led.red(0).blue(255).update();
  }
#endif

  initialisation.Arduino_SPI_init();

  vTaskDelay(pdMS_TO_TICKS(500));
  print_config();

  initialisation.MAX178X_init_sequence();
  xTaskCreatePinnedToCore(SPI_Task, "SPI_Task", 8192, NULL, 1, &spiTaskHandle, 0);

  Serial.println("Waiting for good battery data");

  vTaskDelay(pdMS_TO_TICKS(3000));
  for (;;) {
    wm.process();
    if (!has_errors(&inverter_data)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  };

  Serial.println("Data for inverter OK - Starting CAN");

  rx_queue = xQueueCreate(100, sizeof(twai_message_t));
  tx_queue = xQueueCreate(50, sizeof(twai_message_t));
  xTaskCreatePinnedToCore(TWAI_Task, "TWAI_Task", 4096, NULL, 6, &twaiTaskHandle, 0);
#ifdef MCP2515_CS
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  xTaskCreatePinnedToCore(SPI_CAN_Task, "SPI_CAN_Task", 4096, NULL, 6, &canTaskHandle, 0);
#endif
  xTaskCreatePinnedToCore(TWAI_Processing_Task, "TWAI_Processing_Task", 4096, NULL, 4, &twaiprocessTaskHandle, 0);

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
  wm.process();
  ArduinoOTA.handle();
}  // App in threads only

#ifdef MCP2515_CS
void SPI_CAN_Task(void *pvParameters) {
  Serial.println("SPI_CAN_Task Start");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1));
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      // convert MCP canframe to twai struct and push to processing queue
      canFrame.identifier = canMsg.can_id;
      canFrame.data_length_code = canMsg.can_dlc;
      for (int i = 0; i < canMsg.can_dlc; i++) {
        canFrame.data[i] = canMsg.data[i];
      }
      if (xQueueSend(rx_queue, (void *)&canFrame, pdMS_TO_TICKS(10)) != pdPASS) {
        Serial.println("MCP2515 -> rx_queue overflow");
      }
    }
  }
}
#endif

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
#ifdef RGBLED
          led.red(255).blue(0).green(0).update();
#endif
          break;
        case 3:
          if ((contactor_time + 1000) < millis()) { break; }  // slow transitions to 1000ms
          switch (contactor) {
            case 0:
              contactor = PRECHARGE;
              contactor_time = millis();
              Serial.println("Contactors commanded Precharge On");
              digitalWrite(PRECHARGE_PIN, contactor & 1);
              digitalWrite(MAIN_CONTACTOR_PIN, contactor >> 1);
#ifdef RGBLED
              led.red(255).green(255).update();
#endif
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
#ifdef RGBLED
              led.red(0).green(255).update();
#endif

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
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start driver");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
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
          Serial.println("TWAI -> rx_queue overflow");
        }
      }
    }
    // Check if tx messages are queued and transmit on bus
    while (xQueueReceive(tx_queue, (void *)&rxFrame, 5) == pdPASS) {
      if (can_tx_stop) { continue; }
#ifdef RGBLED
      led.clear();
#endif
#ifdef CAN_TX_INTERSPACE_MS
      vTaskDelay(pdMS_TO_TICKS(CAN_TX_INTERSPACE_MS));  // inter-txframe delay
#endif
      vTaskDelay(pdMS_TO_TICKS(1));  // inter-txframe delay
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
#ifdef RGBLED
    led.update();
#endif
  }
  vTaskDelete(NULL);
}

void SPI_Task(void *pvParameters) {
  Maxim *maxim = Maxim::build();
  Serial.println("SPI_Task Maxim started");

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
    print_readings = can_debug = false;
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
    Serial.println("Shutting down\n\rKilling spi...");
    vTaskDelete(spiTaskHandle);
    digitalWrite(SHDNL_MAX17841_1, LOW);
    Serial.println("Rebooting ESP32");
    ESP.restart();  // Reboot the ESP
  }
}

void saveParamsCallback() {
  strcpy(mqtt_config.server, custom_mqtt_server.getValue());
  strcpy(mqtt_config.user, custom_mqtt_user.getValue());
  strcpy(mqtt_config.password, custom_mqtt_password.getValue());
  strcpy(mqtt_config.topic, custom_mqtt_topic.getValue());
  Serial.println("MQTT params stored");
  saveMQTTConfig();

  unsigned char max_soc = atoi(custom_max_soc.getValue());
  unsigned char min_soc = atoi(custom_min_soc.getValue());
  if ((max_soc <= 100) && (!max_soc) && (min_soc < max_soc) && (!min_soc)) {
    config.max_soc = max_soc;
    config.min_soc = min_soc;
    Serial.println("SoC limit params changed");
  }

  unsigned char num_modules = atoi(custom_num_slaves.getValue());
  unsigned char num_cells = atoi(custom_num_cells.getValue());
  if ((num_modules <= 32) && (!num_modules) && (num_cells <= 14) && (!num_cells)) {
    config.num_modules = num_modules;
    config.cells_per_slave = num_cells;
    Serial.println("Module array params changed");
  }
  saveConfig();
  vTaskDelay(pdMS_TO_TICKS(1000));
  handle_keypress('r');  // Gracefully reboot controller
}

ESP32MQTTClient mqttClient;
void STA_task(void *pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(10));  // low pri-task
  }
  unsigned long mqtt_time = 0;
  Serial.printf("MQTT Task: Connecting to %s @ %s\n\r", mqtt_config.user, mqtt_config.server);
  mqttClient.setURI(mqtt_config.server, mqtt_config.user, mqtt_config.password);
  mqttClient.setMqttClientName(espClientName);
  mqttClient.enableLastWillMessage("lwt", "Maxim offline");
  mqttClient.setKeepAlive(30);
  mqttClient.loopStart();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10));  // low pri-task
    if ((mqtt_time + 5000) > millis()) { continue; }
    mqtt_time = millis();
    if (inverter_data.soc == 0) { continue; }
    char *jsonChar = BMSDataToJson(inverter_data);
    mqttClient.publish(mqtt_config.topic, jsonChar, 0, false);
// Serial.printf("MQTT: %s = %s\n\r", custom_mqtt_topic.getValue(), jsonChar);
#ifdef RGBLED
    led.red(255).update();
    vTaskDelay(pdMS_TO_TICKS(500));  // low pri-task
    led.red(0).update();
#endif
    free(jsonChar);
  }
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

char *BMSDataToJson(const BMS_Data &inverter_data) {
  JsonDocument doc;
  doc["timestamp"] = inverter_data.timestamp;

  JsonArray die_temp = doc["die_temp"].to<JsonArray>();
  JsonArray errors = doc["errors"].to<JsonArray>();
  errors.add(inverter_data.errors[inverter_data.num_modules + 1]);  // global errors first [0]
  for (int i = 0; i < inverter_data.num_modules; i++) {
    die_temp.add(inverter_data.die_temp[i]);
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

bool setup_wifi() {
  uint64_t chipid = ESP.getEfuseMac();
  // Sets unique name
  snprintf(espClientName, 50, "ESP32MAXIM-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  ArduinoOTA.setHostname(espClientName);

  custom_mqtt_server.setValue(mqtt_config.server, strlen(mqtt_config.server));
  custom_mqtt_user.setValue(mqtt_config.user, strlen(mqtt_config.user));
  custom_mqtt_password.setValue(mqtt_config.password, strlen(mqtt_config.password));
  custom_mqtt_topic.setValue(mqtt_config.topic, strlen(mqtt_config.topic));
  custom_num_slaves.setValue(String(config.num_modules).c_str(), strlen(String(config.num_modules).c_str()));
  custom_num_cells.setValue(String(config.cells_per_slave).c_str(), strlen(String(config.num_modules).c_str()));
  custom_max_soc.setValue(String(config.max_soc).c_str(), strlen(String(config.max_soc).c_str()));
  custom_min_soc.setValue(String(config.min_soc).c_str(), strlen(String(config.min_soc).c_str()));

  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(60);
  wm.setMinimumSignalQuality(15);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_num_slaves);
  wm.addParameter(&custom_num_cells);
  wm.addParameter(&custom_max_soc);
  wm.addParameter(&custom_min_soc);

  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(saveParamsCallback);

  if (wm.autoConnect(espClientName)) {
    Serial.println("WiFi STA connected");
    wm.startWebPortal();
    return true;
  } else {
    Serial.println("WiFi Configportal running");
    wm.startConfigPortal();
    return false;
  }
}

void ota() {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(10));  // low pri-task
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}