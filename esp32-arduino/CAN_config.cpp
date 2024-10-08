#include <sys/_stdint.h>
#include <string.h>
#include "Arduino.h"
#include "CAN_config.h"
#include <freertos/FreeRTOS.h>
#include "esp_err.h"
#include "esp_log.h"
#include <esp_task_wdt.h>



void CanBus::summary(BMS_Data *local_result, bool show) {
  print_readings = show;
  if (!print_readings) {
    return;
  }
  const char *chargeStatus = (CANmilliamps < 0) ? "Charging" : "Discharging";
  Serial.printf(
    "\n\r"
    "Cell Min: %dmV Max: %dmV Delta: %d Balancing: %d \n\r"
    "Pack Volts: %.1fV SoC: %.1f Current: %.1fA %s. \n\r"
    "Max Charge Rate: %dA Max Discharge Rate: %dA \n\r",
    local_result->cell_mv_min,
    local_result->cell_mv_max,
    (local_result->cell_mv_max - local_result->cell_mv_min),
    local_result->num_bal_cells,
    local_result->pack_volts,
    local_result->soc,
    CANmilliamps * 0.001,
    chargeStatus,
    max_charge(local_result),
    max_discharge(local_result));

  Serial.printf("Temp Min: %.2fºC Max: %.2fºC \n\n\r", local_result->cell_temp_min, local_result->cell_temp_max);
}

char CanBus::canread(twai_message_t rxFrame, QueueHandle_t tx_queue, BMS_Data *local_result) {
  inverter_sent = false;
  no_errors = true;

  if (!timestamp_in_range(local_result, DATA_TIMEOUT_MS)) {
    no_errors = false;
    long age = millis() - local_result->timestamp;
    Serial.printf("Last data reading is too old (%dms)\n\r", age);
    return (inverter_sent | (no_errors << 1));
  }

  switch (rxFrame.identifier) {
    case CAB300a:
    case CAB300b:
    case CAB300c:
      CAB300(rxFrame);
      break;
    case SAMSUNG_SDI:
      SAMSUNGSDI(rxFrame);
      break;
  }

  if (has_errors(local_result)) {
    no_errors = false;
    Serial.println("Inv CAN send rejected - errors in battery data");
    return (inverter_sent | (no_errors << 1));
  }

  inverter_sent = true;
  switch (rxFrame.identifier) {
    case FOX:
      BMS_ComHV(local_result, rxFrame, tx_queue);
      break;  // ignore other frames on this ID
    case GW:
      BMS_ComHV_GW(local_result, rxFrame, tx_queue);
      break;
    case LV_INVERTER:
      BMS_ComLV(local_result, tx_queue);
      break;
    default:
      // Unwanted CAN ID
      break;
  }
  return (inverter_sent | (no_errors << 1));
}

void CanBus::CAB300(twai_message_t frame) {
  uint32_t inbox = 0;

  for (int i = 0; i < 4; i++) {
    inbox = (inbox << 8) | frame.data[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000) {
    CANmilliamps -= 0x80000000;
  } else {
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  }
}

void CanBus::SAMSUNGSDI(twai_message_t frame)  // Samsung Sensor
{
  uint r = 0;
  byte b0 = 0xff;
  if ((frame.data[2] & 0x80) != 0)
    r |= b0 << 16;
  r |= frame.data[1] << 8;
  r |= frame.data[0];
  CANmilliamps = r;
}

void CanBus::BMS_ComLV(BMS_Data *result, QueueHandle_t tx_queue) {  // communication with LV BMS system over CAN
  if (print_readings) {
    Serial.println("CAN SEND ================> Send LV Inverter Data");
  }
  twai_message_t tx_msg = { 0 };
  tx_msg.identifier = 0x359;
  tx_msg.extd = 0;
  tx_msg.data_length_code = 7;
  tx_msg.data[0] = 0x00;  // protection to be translated later date
  tx_msg.data[1] = 0x00;  // protection to be translated later date
  tx_msg.data[2] = 0x00;  // protection to be translated later date
  tx_msg.data[3] = 0x00;  // protection to be translated later date
  tx_msg.data[4] = 0x03;  // number of modules fixed for now
  tx_msg.data[5] = 0x50;
  tx_msg.data[6] = 0x4E;
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x351;
  tx_msg.data_length_code = 8;
  tx_msg.data[0] = lowByte(short(max_pack_voltage_cutoff()));
  tx_msg.data[1] = highByte(short(max_pack_voltage_cutoff()));
  tx_msg.data[2] = lowByte(max_charge(result));
  tx_msg.data[3] = highByte(max_charge(result));
  tx_msg.data[4] = lowByte(max_discharge(result));
  tx_msg.data[5] = highByte(max_discharge(result));
  tx_msg.data[6] = lowByte(short(low_pack_voltage_cutoff()));
  tx_msg.data[7] = highByte(short(low_pack_voltage_cutoff()));
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x355;
  tx_msg.data_length_code = 4;
  tx_msg.data[0] = lowByte(short(result->soc));
  tx_msg.data[1] = highByte(short(result->soc));
  tx_msg.data[2] = lowByte(99);   // static for now
  tx_msg.data[3] = highByte(99);  // static for now
  tx_msg.data[4] = 0x00;
  tx_msg.data[5] = 0x00;
  tx_msg.data[6] = 0x00;
  tx_msg.data[7] = 0x00;
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x356;
  tx_msg.data_length_code = 6;
  tx_msg.data[0] = lowByte(short(result->pack_volts * 10));
  tx_msg.data[1] = highByte(short(result->pack_volts * 10));
  tx_msg.data[2] = lowByte(short(CANmilliamps * 0.01));
  tx_msg.data[3] = highByte(short(CANmilliamps * 0.01));
  tx_msg.data[4] = lowByte(int16_t(result->cell_temp_max * 10));   //temp celcius
  tx_msg.data[5] = highByte(int16_t(result->cell_temp_max * 10));  //temp celcius
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x35C;
  tx_msg.data_length_code = 2;
  tx_msg.data[0] = 0xC0;  // fixed charge and discharge enable for verification
  tx_msg.data[1] = 0x00;
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x35E;
  tx_msg.data_length_code = 8;
  tx_msg.data[0] = 0x50;  // P
  tx_msg.data[1] = 0x59;  // Y
  tx_msg.data[2] = 0x4C;  // L
  tx_msg.data[3] = 0x4F;  // O
  tx_msg.data[4] = 0x4E;  // N
  tx_msg.data[5] = 0x20;
  tx_msg.data[6] = 0x20;
  tx_msg.data[7] = 0x20;
  send_to_queue(tx_queue, &tx_msg);
}

void CanBus::BMS_ComHV(BMS_Data *result, twai_message_t rxFrame, QueueHandle_t tx_queue) {
  num_fox_slaves = char(result->pack_volts * 0.0172) + 1;

  if (filter_data_array((const char *)rxFrame.data, Fox_TimeStamp, 2)) {
    if (print_readings) {
      Serial.printf("Inverter Time: 20%d-%02d-%02d %02d:%02d:%02d\n\r", rxFrame.data[2], rxFrame.data[3], rxFrame.data[4], rxFrame.data[5], rxFrame.data[6], rxFrame.data[7]);
    }
    return;
  }
  if (filter_data_array((const char *)rxFrame.data, Fox_Version_Data, 8)) {
    BMS_ComHV_Version(result, tx_queue);
    return;
  }
  if (filter_data_array((const char *)rxFrame.data, Fox_Pack_Data, 8)) {
    BMS_ComHV_Pack_Data(result, tx_queue);
    return;
  }
  if (filter_data_array((const char *)rxFrame.data, Fox_Init_Data, 8)) {
    if (!announce) {
      BMS_ComHV_Announce(result, tx_queue);
      announce = true;
      return;
    }
    BMS_ComHV_Data(result, tx_queue);
    inverter_sent = true;
  }
}

void CanBus::BMS_ComHV_Announce(BMS_Data *result, QueueHandle_t tx_queue) {
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Inverter Announce");
  }
  twai_message_t tx_msg = { 0 };
  tx_msg.identifier = 0x100A001;
  tx_msg.extd = 1;
  tx_msg.data_length_code = 0;  // empty frame
  send_to_queue(tx_queue, &tx_msg);
}

void CanBus::BMS_ComHV_Version(BMS_Data *result, QueueHandle_t tx_queue) {  // communication with BMS system over CAN
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Inverter Version Data");
  }
  static const char data[27][8] = {
    { 0x00, 0x36, 0x30, 0x42, 0x42, 0x48, 0x56, 0x32 },
    { 0x00, 0x30, 0x32, 0x32, 0x48, 0x41, 0x30, 0x32 },
    { 0x00, 0x38, 0x32, 0x48, 0x41, 0x00, 0x00, 0x00 },
    { 0x01, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x01, 0x30, 0x32, 0x36, 0x52, 0x43, 0x30, 0x34 },
    { 0x01, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x02, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x02, 0x30, 0x32, 0x36, 0x52, 0x43, 0x30, 0x35 },
    { 0x02, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x03, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x03, 0x31, 0x32, 0x35, 0x51, 0x41, 0x31, 0x32 },
    { 0x03, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x04, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x04, 0x31, 0x32, 0x35, 0x55, 0x46, 0x30, 0x39 },
    { 0x04, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x05, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x05, 0x30, 0x32, 0x36, 0x42, 0x45, 0x30, 0x37 },
    { 0x05, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x06, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x06, 0x30, 0x32, 0x36, 0x42, 0x45, 0x30, 0x37 },
    { 0x06, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x07, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x07, 0x30, 0x32, 0x36, 0x42, 0x45, 0x30, 0x37 },
    { 0x07, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x08, 0x36, 0x30, 0x32, 0x48, 0x32, 0x36, 0x33 },
    { 0x08, 0x30, 0x32, 0x36, 0x42, 0x45, 0x30, 0x37 },
    { 0x08, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
  };

  twai_message_t tx_msg = { 0 };
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;

  unsigned int id[] = { 0x1881, 0x1882, 0x1883 };
  for (char pack = 0; pack < ((num_fox_slaves * 3) + 1); (pack = pack + 3)) {
    for (char j = 0; j < 3; j++) {
      tx_msg.identifier = id[j];
      for (char i = 0; i < tx_msg.data_length_code; i++) {
        tx_msg.data[i] = data[pack + j][i];
      }
      send_to_queue(tx_queue, &tx_msg);
    }
  }
}

void CanBus::BMS_ComHV_Cell_Data(BMS_Data *result, QueueHandle_t tx_queue) {
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Inverter Cell Voltages");
  }
  int id = 0xC1D;
  twai_message_t tx_msg = { 0 };
  tx_msg.data_length_code = 8;
  for (int cell = 0; cell < (result->num_modules * config.cells_per_slave); cell = (cell + 4)) {
    tx_msg.identifier = id + cell;
    tx_msg.data[0] = lowByte(result->cell_millivolts[cell]);
    tx_msg.data[1] = highByte(result->cell_millivolts[cell]);
    tx_msg.data[2] = lowByte(result->cell_millivolts[cell + 1]);
    tx_msg.data[3] = highByte(result->cell_millivolts[cell + 1]);
    tx_msg.data[4] = lowByte(result->cell_millivolts[cell + 2]);
    tx_msg.data[5] = highByte(result->cell_millivolts[cell + 2]);
    tx_msg.data[6] = lowByte(result->cell_millivolts[cell + 3]);
    tx_msg.data[7] = highByte(result->cell_millivolts[cell + 3]);
    send_to_queue(tx_queue, &tx_msg);
  }
}

void CanBus::BMS_ComHV_Temp_Data(BMS_Data *result, QueueHandle_t tx_queue) {
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Temp Data");
  }
  twai_message_t tx_msg = { 0 };
  tx_msg.data_length_code = 8;
  for (int id = 0xD21; id < ((8 * num_fox_slaves) + 0xD21); id = (id + 8)) {
    tx_msg.identifier = id;
    tx_msg.data[0] = lowByte((result->cell_temp[0] + 50));
    tx_msg.data[1] = highByte((result->cell_temp[0] + 50));
    tx_msg.data[2] = lowByte((result->cell_temp[1] + 50));
    tx_msg.data[3] = highByte((result->cell_temp[1] + 50));
    tx_msg.data[4] = lowByte((result->cell_temp[2] + 50));
    tx_msg.data[5] = highByte((result->cell_temp[2] + 50));
    tx_msg.data[6] = lowByte((result->cell_temp[3] + 50));
    tx_msg.data[7] = highByte((result->cell_temp[3] + 50));
    send_to_queue(tx_queue, &tx_msg);
  }
}

void CanBus::BMS_ComHV_Pack_Data(BMS_Data *result, QueueHandle_t tx_queue) {
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Slave Data");
  }
  twai_message_t tx_msg = { 0 };
  const unsigned int id = 0xC05;
  tx_msg.data_length_code = 8;
  for (char pack = 0; pack < num_fox_slaves; (pack = pack + 3)) {
    tx_msg.identifier = id + pack;
    tx_msg.data[0] = lowByte(short(CANmilliamps * 0.01));  // mA => A * 10 scale
    tx_msg.data[1] = highByte(short(CANmilliamps * 0.01));
    tx_msg.data[2] = (signed char)(result->cell_temp_max);  // i8 high temp
    tx_msg.data[3] = (signed char)(result->cell_temp_min);  // i8 low temp
    tx_msg.data[4] = char(result->soc);
    tx_msg.data[5] = char(max_charge(result) * 10);
    tx_msg.data[6] = lowByte(short(result->pack_volts / num_fox_slaves));  // simulated voltage
    tx_msg.data[7] = highByte(short(result->pack_volts / num_fox_slaves));
    send_to_queue(tx_queue, &tx_msg);
  }
}

void CanBus::BMS_ComHV_Data(BMS_Data *result, QueueHandle_t tx_queue) {
  // communication with BMS system over CAN
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV Inverter Data");
  }
  twai_message_t tx_msg = { 0 };
  tx_msg.identifier = 0x1872;
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;
  tx_msg.data[0] = lowByte(short(max_pack_voltage_cutoff() * 10));  // max Battery Volts    //max Battery Volts
  tx_msg.data[1] = highByte(short(max_pack_voltage_cutoff() * 10));
  tx_msg.data[2] = lowByte(short(low_pack_voltage_cutoff() * 10));  // min battery Volts
  tx_msg.data[3] = highByte(short(low_pack_voltage_cutoff() * 10));
  tx_msg.data[4] = lowByte(max_charge(result) * 10);  // max charge rate
  tx_msg.data[5] = highByte(max_charge(result) * 10);
  tx_msg.data[6] = lowByte(max_discharge(result) * 10);  // max discharge rate
  tx_msg.data[7] = highByte(max_discharge(result) * 10);
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x1873;
  tx_msg.data[0] = lowByte(short(result->pack_volts * 10));  // Battery Voltage for series connected
  tx_msg.data[1] = highByte(short(result->pack_volts * 10));
  tx_msg.data[2] = lowByte(short(CANmilliamps * 0.01));  //
  tx_msg.data[3] = highByte(short(CANmilliamps * 0.01));
  tx_msg.data[4] = lowByte(short(result->soc));  // SOC
  tx_msg.data[5] = highByte(short(result->soc));
  tx_msg.data[6] = lowByte(short(config.slave_kwh * result->num_modules * (result->soc) + 1));  // Battery Kwh
  tx_msg.data[7] = highByte(short(config.slave_kwh * result->num_modules * (result->soc) + 1));
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x1874;                                   // limits
  tx_msg.data[0] = lowByte(short(result->cell_temp_max * 10));  // Battery Temperature // cell temp high  deg C * 10
  tx_msg.data[1] = highByte(short(result->cell_temp_max * 10));
  tx_msg.data[2] = lowByte(short(result->cell_temp_min * 10));  // Battery Temperature// cell temp low  deg C * 10
  tx_msg.data[3] = highByte(short(result->cell_temp_min * 10));
  tx_msg.data[4] = lowByte(short(config.min_cell_millivolts * 0.01));
  tx_msg.data[5] = highByte(short(config.min_cell_millivolts * 0.01));
  tx_msg.data[6] = lowByte(short(config.max_cell_millivolts * 0.01));
  tx_msg.data[7] = highByte(short(config.max_cell_millivolts * 0.01));
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x1875;
  tx_msg.data[0] = lowByte(short(result->cell_temp_max * 10.0));  // Battery Temperature
  tx_msg.data[1] = highByte(short(result->cell_temp_max * 10.0));
  tx_msg.data[2] = (1 < num_fox_slaves) - 1;
  tx_msg.data[3] = num_fox_slaves;
  tx_msg.data[4] = announce;  // contactor !
  tx_msg.data[5] = (0X00);
  tx_msg.data[6] = (0X00);
  tx_msg.data[7] = (0X00);
  send_to_queue(tx_queue, &tx_msg);

  bool stop_charging = ((0 == max_charge(result)) || (char(result->soc) >= 100) || (result->cell_mv_max > config.max_cell_millivolts));
  if (stop_charging) {
    Serial.printf("Charge inhibit bit: Max charge: %d | SOC >100: %d | Cell overvolt: %d    ", stop_charging, (0 == max_charge(result)), (100 == char(result->soc)), (result->cell_mv_max > config.max_cell_millivolts));
  }
  tx_msg.identifier = 0x1876;  // permitted ranges
  tx_msg.data[0] = char(stop_charging);
  tx_msg.data[1] = (0X00);
  tx_msg.data[2] = lowByte(short(result->cell_mv_max));
  tx_msg.data[3] = highByte(short(result->cell_mv_max));
  tx_msg.data[4] = (0X00);
  tx_msg.data[5] = (0X00);
  tx_msg.data[6] = lowByte(short(result->cell_mv_min));
  tx_msg.data[7] = highByte(short(result->cell_mv_min));
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x1877;
  tx_msg.data[0] = (0x00);
  tx_msg.data[1] = (0X00);
  tx_msg.data[2] = 0x00;
  tx_msg.data[3] = 0x00;
  tx_msg.data[4] = (fox_counter == 0) ? (0x52) : (0x84);
  tx_msg.data[5] = (0X00);
  tx_msg.data[6] = (fox_counter == 0) ? (11) : (0x17);
  tx_msg.data[7] = (fox_counter == 0) ? (0x01) : (fox_counter * 0x10);
  send_to_queue(tx_queue, &tx_msg);
  fox_counter = (fox_counter + 1) % (num_fox_slaves + 1);  //8 slaves + 1 bms

  tx_msg.identifier = 0x1878;
  tx_msg.data[0] = lowByte(short(result->pack_volts * 10));
  tx_msg.data[1] = highByte(short(result->pack_volts * 10));
  tx_msg.data[2] = (0X00);
  tx_msg.data[3] = (0X00);
  tx_msg.data[4] = (0X10);  // total watt hours
  tx_msg.data[5] = (0X27);
  tx_msg.data[6] = (0X00);
  tx_msg.data[7] = (0X00);
  send_to_queue(tx_queue, &tx_msg);

  char status = char((CANmilliamps * 0.01) < 0) ? CHARGE : (((CANmilliamps * 0.01) > 0) ? DISCHARGE : IDLE);
  tx_msg.identifier = 0x1879;
  tx_msg.data[0] = 0;
  tx_msg.data[1] = status;
  tx_msg.data[2] = 0;
  tx_msg.data[3] = 0;
  tx_msg.data[4] = 0;
  tx_msg.data[5] = 0;
  tx_msg.data[6] = 0;
  tx_msg.data[7] = 0;
  send_to_queue(tx_queue, &tx_msg);
}

void CanBus::BMS_ComHV_GW(BMS_Data *result, twai_message_t rxFrame, QueueHandle_t tx_queue) {
  switch (rxFrame.data[0]) {
    case 0:
      BMS_GWHV_Data(result, tx_queue);
      break;
    default:
      BMS_GWHV_Info(result, tx_queue);
      break;
  }
}

void CanBus::BMS_GWHV_Data(BMS_Data *result, QueueHandle_t tx_queue) {
  // info https://github.com/stuartpittaway/diyBMSv4ESP32/blob/9a0a3ed26f2dc973d3faf3e2799a6688da3ab13e/ESPController/src/pylonforce_canbus.cpp
  // src https://github.com/JoeMudr/JoeMBMC/tree/072d2beff6c75bfb339139ffc936c3f1b63cede1

  const char batteryID = 0;

  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV GW Inverter Data");
  }
  twai_message_t tx_msg = { 0 };
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;

  int current = (result->milliamps * 0.01) + 3000;  // A * 10 + 3000
  tx_msg.identifier = 0x4210 + batteryID;
  tx_msg.data[0] = lowByte(short(result->pack_volts) * 10);
  tx_msg.data[1] = highByte(short(result->pack_volts) * 10);
  tx_msg.data[2] = lowByte(short(current));  // [ToDo] values > ~6500A will overflow!
  tx_msg.data[3] = highByte(short(current));
  tx_msg.data[4] = lowByte(short(result->cell_temp_max) + 100);
  tx_msg.data[5] = highByte(short(result->cell_temp_max) + 100);
  tx_msg.data[6] = char(result->soc);
  tx_msg.data[7] = 100;  //soh
  send_to_queue(tx_queue, &tx_msg);

  int max_pack_v = max_pack_voltage_cutoff() * 10;
  int min_pack_v = low_pack_voltage_cutoff() * 10;
  int charge = (max_charge(result) * 10) + 3000;
  int discharge = (max_discharge(result) * 10) + 3000;
  tx_msg.identifier = 0x4220 + batteryID;
  tx_msg.data[0] = lowByte(short(max_pack_v));  // max pack v
  tx_msg.data[1] = highByte(short(max_pack_v));
  tx_msg.data[2] = lowByte(short(min_pack_v));  // min pack v
  tx_msg.data[3] = highByte(short(min_pack_v));
  tx_msg.data[4] = lowByte(short(charge));
  tx_msg.data[5] = highByte(short(charge));
  tx_msg.data[6] = lowByte(short(discharge));
  tx_msg.data[7] = highByte(short(discharge));
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x4230 + batteryID;
  tx_msg.data[0] = lowByte(short(result->cell_mv_min));
  tx_msg.data[1] = highByte(short(result->cell_mv_min));
  tx_msg.data[2] = lowByte(short(result->cell_mv_max));
  tx_msg.data[3] = highByte(short(result->cell_mv_max));
  tx_msg.data[4] = 0;  // MAX Single Battery Cell Voltage Number [ToDo]
  tx_msg.data[5] = 0;  // MAX Single Battery Cell Voltage Number [ToDo]
  tx_msg.data[6] = 1;  // MIN Single Battery Cell Voltage Number [ToDo]
  tx_msg.data[7] = 0;  // MIN Single Battery Cell Voltage Number [ToDo]
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x4240 + batteryID;
  tx_msg.data[0] = lowByte(short(result->cell_temp_max * 10 + 100));
  tx_msg.data[1] = lowByte(short(result->cell_temp_max * 10 + 100));
  tx_msg.data[2] = lowByte(short(result->cell_temp_min * 10 + 100));
  tx_msg.data[3] = lowByte(short(result->cell_temp_min * 10 + 100));
  tx_msg.data[4] = 0;  // MAX Single Battery Cell Temperature Number [ToDo]
  tx_msg.data[5] = 0;  // MAX Single Battery Cell Temperature Number [ToDo]
  tx_msg.data[6] = 1;  // MIN Single Battery Cell Temperature Number [ToDo]
  tx_msg.data[7] = 0;  // MIN Single Battery Cell VolTemperaturetage Number [ToDo]
  send_to_queue(tx_queue, &tx_msg);

  // Errors & Alarms [ToDo from Maxim errors)
  uint16_t tmpAlarm = 0;
  uint16_t tmp = 0;
  // tmp = (alarm[0] & 0b00010000) >> 4;  // Bit0: Cell low Volt
  // tmpAlarm |= tmp;
  // tmp = (alarm[0] & 0b00000100) >> 1;  // Bit1: Cell high Volt
  // tmpAlarm |= tmp;
  // tmp = (alarm[0] & 0b00010000) >> 2;  // Bit2: Discharge low Volt
  // tmpAlarm |= tmp;
  // tmp = (alarm[0] & 0b00000100);  // Bit3: Charge high Volt
  // tmpAlarm |= tmp;
  // tmp = (alarm[1] & 0b00000001) << 3;  // Bit4: Charge Cell low Temp
  // tmpAlarm |= tmp;
  // tmp = (alarm[0] & 0b01000000) >> 2;  // Bit5: Charge Cell high Temp
  // tmpAlarm |= tmp;
  // tmp = (alarm[1] & 0b00000001) << 5;  // Bit6: Discharge Cell low Temp
  // tmpAlarm |= tmp;
  // tmp = (alarm[0] & 0b01000000);  // Bit7: Discharge Cell high Temp
  // tmpAlarm |= tmp;
  //tmp = 0b00000000; // Bit8: Charge Overcurrent [ToDo]
  //tmpAlarm |= tmp;
  //tmp = 0b01000000; // Bit9: Discharge Overcurrent [ToDo]
  //tmpAlarm |= tmp;

  char status = char((CANmilliamps * 0.01) < 0) ? 1 : (((CANmilliamps * 0.01) > 0) ? 2 : 3);

  tx_msg.identifier = 0x4250 + batteryID;
  tx_msg.data[0] = status;              // bit0-2: 0 = sleep, 1 = Charge, 2 = Discharge, 3 = Idle; Bit3: Force Charge; Bit4: Force Balance Charge; Bit5-7: Reserved; [ToDo]
  tx_msg.data[1] = 0;                   // Cycle Period? [ToDo]
  tx_msg.data[2] = 0;                   // Cycle Period? [ToDo]
  tx_msg.data[3] = 0;                   // Error / Faults [ToDo] not implemented yet
  tx_msg.data[4] = lowByte(tmpAlarm);   // Alarm
  tx_msg.data[5] = highByte(tmpAlarm);  // Alarm
  tx_msg.data[6] = 0;                   // Protection same as Alarm? [ToDo]
  tx_msg.data[7] = 0;                   // Protection same as Alarm? [ToDo]
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x4280 + batteryID;
  memset(tx_msg.data, 0, 8);
  bool stop_charging = ((0 == max_charge(result)) || (char(result->soc) >= 100) || (result->cell_mv_max > config.max_cell_millivolts));
  bool stop_discharging = ((0 == max_discharge(result)) || (char(result->soc) <= 0) || (result->cell_mv_min < config.min_cell_millivolts));
  tx_msg.data[0] = stop_charging ? 0 : 0xAA;
  tx_msg.data[2] = stop_discharging ? 0 : 0xAA;
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x4260 + batteryID;  // Module Voltage MIN / MAX & Nr.
  memset(tx_msg.data, 0, 8);
  uint16_t module_voltage = (result->num_modules / result->pack_volts) * 1000;
  tx_msg.data[0] = lowByte(module_voltage + 1);
  tx_msg.data[1] = highByte(module_voltage + 1);
  tx_msg.data[2] = lowByte(module_voltage);
  tx_msg.data[3] = highByte(module_voltage);
  tx_msg.data[6] = 1;
  send_to_queue(tx_queue, &tx_msg);

  // tx_msg.identifier = 0x4270 + batteryID;  // Module Temp. MIN / MAX & Nr.
}

void CanBus::BMS_GWHV_Info(BMS_Data *result, QueueHandle_t tx_queue) {
  if (print_readings) {
    Serial.println("CAN SEND ================> Send HV GW Inverter Info");
  }
  const char batteryID = 1;
  twai_message_t tx_msg = { 0 };
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;

  tx_msg.identifier = 0x7310 + batteryID;
  tx_msg.data[0] = 0;  // Version 1=A, 2=B
  tx_msg.data[1] = 0;  // reserved
  tx_msg.data[2] = 3;  // Hardware Version e.g. 3.2
  tx_msg.data[3] = 2;  // Hardware Version e.g. 3.2
  tx_msg.data[4] = 3;  // Software Version e.g. 3.4 (03/2024)
  tx_msg.data[5] = 4;  // Software Version e.g. 3.4 (03/2024)
  tx_msg.data[6] = 0;  // Software Version ??
  tx_msg.data[7] = 0;  // Software Version ??
  send_to_queue(tx_queue, &tx_msg);

  tx_msg.identifier = 0x7320 + batteryID;
  int num_cells = config.num_modules * config.cells_per_slave;
  tx_msg.data[0] = lowByte(num_cells);
  tx_msg.data[1] = highByte(num_cells);
  tx_msg.data[2] = config.num_modules;
  tx_msg.data[3] = config.cells_per_slave;
  tx_msg.data[4] = lowByte(((config.max_cell_millivolts - config.min_cell_millivolts) * num_cells));
  tx_msg.data[5] = highByte(((config.max_cell_millivolts - config.min_cell_millivolts) * num_cells));
  tx_msg.data[6] = lowByte(short(config.slave_kwh * 1000));
  tx_msg.data[7] = highByte(short(config.slave_kwh * 1000));
  send_to_queue(tx_queue, &tx_msg);


  tx_msg.identifier = 0x7330 + batteryID;
  tx_msg.data[0] = 'z';
  tx_msg.data[1] = 'z';
  tx_msg.data[2] = 'z';
  tx_msg.data[3] = 'z';
  tx_msg.data[4] = 'z';
  tx_msg.data[5] = 'z';
  tx_msg.data[6] = 'z';
  tx_msg.data[7] = 'z';
  send_to_queue(tx_queue, &tx_msg);


  tx_msg.identifier = 0x7340 + batteryID;
  tx_msg.data[0] = 0;
  tx_msg.data[1] = 0;
  tx_msg.data[2] = 0;
  tx_msg.data[3] = 0;
  tx_msg.data[4] = 0;
  tx_msg.data[5] = 0;
  tx_msg.data[6] = 0;
  tx_msg.data[7] = 0;
  send_to_queue(tx_queue, &tx_msg);
}


// Function to compare two data arrays
bool filter_data_array(const char *arr1, const char *arr2, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (arr1[i] != arr2[i]) {
      return false;
    }
  }
  return true;
}

short max_charge(const BMS_Data *result) {
  if ((result->soc >= config.max_soc) || (config.max_cell_millivolts < result->cell_mv_max)) {
    return 0;  // disable charging
  }
  if (result->soc > config.max_soc - 5) {
    return short((config.max_soc - result->soc) * config.max_charge / 5);  // taper charging nearing max soc - stops high current hammering
  }
  return short(config.max_charge);
}

short max_discharge(const BMS_Data *result) {
  if ((result->soc <= config.min_soc) || (config.min_cell_millivolts > result->cell_mv_min)) {
    return short(0);  // disable discharging
  }
  if (result->soc < 10) {
    return short(config.max_discharge * result->soc / 10);  // taper discharging under 10% soc
  }
  return short(config.max_discharge);
}

void send_to_queue(QueueHandle_t tx_queue, twai_message_t *tx_msg) {
  if (xQueueSend(tx_queue, (void *)tx_msg, pdMS_TO_TICKS(10)) != pdPASS) {
    Serial.printf("Failed to queue message for transmission\n\r");
  }
}

bool check(const BMS_Data *result) {
  // short cell_data_crc = values_crc(result);  // validate cell crc with warning count
  // if (cell_data_crc == last_data_crc) {
  //   if (last_data_crc_fail_count < 10) {
  //     last_data_crc_fail_count++;
  //     // this needs moving to a data request only XXX
  //     Serial.printf("Data Warn: Last data reading was identical more than (10) requests at uptime %d secs, error counter %d\10 \n\r", int(millis() * 0.001), last_data_crc_fail_count);
  //   } else {
  //     no_errors = false;
  //     Serial.println("Data Error: Data readings have been identical over 10 cycles, data rejected");
  //     return false;
  //   }
  // }
  // last_data_crc_fail_count--;
  return true;
}