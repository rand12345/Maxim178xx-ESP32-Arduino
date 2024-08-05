# Maxim MAX17841b, MAX17823 and MAX17845 battery emulator

Currently tested on Opal/Peugeot/Citroen (MAX17845), and Range Rover/BMW (MAX17823) lithium packs.

Emulates FoxEss and PylonTech Force H2 CAN bus protocols.

## Requirements 📦

Before you begin, ensure you have the following libraries and tools installed:

- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [ESP32MQTTClient](https://github.com/cyijun/ESP32MQTTClient)
- [WiFiManager](https://github.com/tzapu/WiFiManager)
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel/)
- [Espressif ESP32 3.0.x board manager](https://dl.espressif.com/dl/package_esp32_index.json)
- [MCP2515 CAN \(Optional\)](https://github.com/autowp/arduino-mcp2515/)

## Setup Instructions 🛠️

1. **Clone the repository**:
    ```bash
    git clone https://github.com/rand12345/Maxim178xx-ESP32-Arduino.git
    cd Maxim178xx-ESP32-Arduino
    ```

2. **Install the required libraries**:
    Open the Arduino IDE and go to `Sketch` > `Include Library` > `Manage Libraries...` and search for the above libraries and install them.

3. **Install the ESP32 board manager**:
    - Open the Arduino IDE and go to `File` > `Preferences`.
    - In the `Additional Board Manager URLs` field, add the following URL:
      ```
      https://dl.espressif.com/dl/package_esp32_index.json
      ```
    - Go to `Tools` > `Board` > `Boards Manager...`, search for `ESP32`, and install the latest version (3.0.x).

4. **Set up `configuration.h`**:
    - Locate the `configuration.h` file in the project directory.
    - Update the file with your specific settings. Below is an example configuration:

    ```c
    // Pack factory settings - some can be edited altered in EEPROM or WIFI menu

    #define F_EXPECTED_PACKS 18               // Important!
    #define F_CELLS_PER_SLAVE 6               // Important!

    // Custom GPIO pins per ESP32 module type
    #if CONFIG_IDF_TARGET_ESP32C6
    #define TWAI_TX_SIGNAL_IDX TWAI0_TX_IDX
    #define RESET_EEPROM_PIN 9
    #define PRECHARGE_PIN 12
    #define MAIN_CONTACTOR_PIN 11
    #define CAN_TX_PIN 2
    #define CAN_RX_PIN 3
    #define sck 5
    #define miso 6
    #define mosi 4
    #define SS1 7                // chip select for MAX17841
    #define SS2 7                // Optional Slave select if Dual UART is used
    #define INT_MAX17841_1 8     // Optional interrupt pin for MAX17841 (unused)
    #define SHDNL_MAX17841_1 10  // Shut down pin for MAX17841 (essential)
    #elif CONFIG_IDF_TARGET_ESP32C3
    #define TWAI_TX_SIGNAL_IDX TWAI_TX_IDX
    #define RESET_EEPROM_PIN 3
    #define PRECHARGE_PIN 9
    #define MAIN_CONTACTOR_PIN 9
    #define CAN_TX_PIN 0
    #define CAN_RX_PIN 1
    #define sck 5
    #define miso 6
    #define mosi 4
    #define SS1 7
    #define SS2 7
    #define INT_MAX17841_1 8
    #define SHDNL_MAX17841_1 10
    #else  // All other ESP32 modules
    #define RESET_EEPROM_PIN 0
    #define PRECHARGE_PIN 9
    #define MAIN_CONTACTOR_PIN 9
    #define CAN_TX_PIN 4
    #define CAN_RX_PIN 2
    #define sck 18
    #define miso 19
    #define mosi 23
    #define SS1 22
    #define SS2 7
    #define INT_MAX17841_1 8
    #define SHDNL_MAX17841_1 5
    #endif
    ```

    Enable SPI MAX2515 CAN bus for current sensors

    ```c
    #define MCP2515_CS 10
    ```

    [ESP32-C6 Dual TWAI TBA](https://github.com/espressif/arduino-esp32/discussions/10107)

5. **Compile and upload the code**:
    - Connect your ESP32 board to your computer.
    - Select the appropriate board and port in the Arduino IDE.
    - Sketch -> Partition Scheme -> Minimal SPIFFS
    - Click on the `Upload` button.

6. **Connect to the ESP32 Access Point**:

1. After the code is uploaded and running, the ESP32 will create an access point named `ESP32MAXIM-XXXX`.
2. On your computer or mobile device, open the WiFi settings.
3. Connect to the `ESP32MAXIM-XXXX` network.

7. **Configure WiFi Credentials**:

1. Once connected to the `ESP32MAXIM-XXXX`, open a web browser.
2. Navigate to `192.168.4.1` in the browser's address bar.
3. You will see a configuration portal where you can select your WiFi network and enter the access point password.
4. After entering the credentials, specify the number of slave modules and cells per slave.
5. Click `Save`

8. **ESP32 Connects to Your WiFi**:

1. The ESP32 will reboot and attempt to connect to the WiFi network you configured.
2. If the connection is successful, you will see "Connected to WiFi!" in the Serial Monitor of the Arduino IDE.
3. Further settings can be modified by accessing the webpage of the device `http://<IP>/`.
4. The IP address of the ESP32 will be visible in the Arduino IDE for OTA updates.

## Usage 🚀

Once the setup is complete, your ESP32 should be ready to use with the configured libraries and settings. Ensure to customize the `configuration.h` file based on your specific requirements and hardware configuration.

## Contributions 🤝

We welcome contributions to enhance this project. Feel free to fork the repository, make your changes, and submit a pull request.

## License 📜

This project is licensed under conditional non-commercial license. See the LICENCE.md file for details.

## Contact 📧

For any queries or support, please open an issue in the GitHub repository.

---

