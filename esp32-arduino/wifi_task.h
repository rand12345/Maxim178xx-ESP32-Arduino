#ifdef WIFI
#include "ESP32MQTTClient.h"
#include "secrets.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/task.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

extern ESP32MQTTClient mqttClient;  // all params are set later

void wifi_start();

#endif