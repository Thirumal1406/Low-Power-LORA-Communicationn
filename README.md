# 📡 Low Power Wireless LoRa Communication System

A robust IoT communication system built on STM32WL microcontrollers featuring adaptive transmit power control, deep sleep power management, and multi-sensor support with reliable acknowledgment-based transmission.

## 🚀 Features

### Transmitter Side
- **Adaptive Transmit Power Control (ATPC)** - Automatically adjusts transmission power based on RSSI feedback
- **Multi-sensor Support** - Compatible with SHT41, DS18B20, MAX31865, and 4-20mA humidity sensors
- **Deep Sleep Management** - Ultra-low power consumption with configurable wake intervals
- **Reliable Transmission** - Acknowledgment-based transmission with automatic retries
- **Battery Monitoring** - Real-time battery voltage and percentage monitoring
- **Timestamp Support** - RTC-based Unix timestamp generation
- **Error Handling** - Comprehensive error detection and recovery mechanisms

### Receiver Side
- **FreeRTOS Integration** - Multi-task architecture for efficient packet processing
- **Automatic Acknowledgment** - Intelligent ACK generation with RSSI feedback
- **CRC Validation** - Packet integrity verification
- **Watchdog Support** - System reliability with independent watchdog timer
- **Queue-based Processing** - Efficient packet handling with FreeRTOS queues

## 📋 Table of Contents

- Hardware Requirements
- Software Dependencies
- Installation
- Configuration
- Usage
- API Reference
- Troubleshooting
- Contributing
- License

## 🛠️ Hardware Requirements

### Transmitter
- STM32WL55xx microcontroller
- One of the following sensors:
  - SHT41 (Temperature/Humidity via I2C)
  - DS18B20 (Temperature via OneWire)
  - MAX31865 (RTD Temperature via SPI)
  - 4-20mA Humidity sensor with shunt resistor
- External RTC (optional, uses internal RTC)
- Battery monitoring circuit

### Receiver
- STM32WL55xx microcontroller
- Serial interface for data output
- External antenna

### Common Requirements
- LoRa antenna (868MHz/915MHz depending on region)
- Power supply (3.3V recommended)
- Debugging interface (ST-Link/J-Link)

## 📚 Software Dependencies

### Arduino Libraries
```
- RadioLib (for LoRa communication)
- STM32LowPower (for deep sleep)
- STM32RTC (for timestamp generation)
- Adafruit_SHT4x (for SHT41 sensor)
- DallasTemperature (for DS18B20 sensor)
- OneWire (for DS18B20 sensor)
- Adafruit_MAX31865 (for RTD sensor)
- FreeRTOS (for receiver task management)
```

### STM32 Core
- STM32duino Core
- STM32WL HAL drivers

## 🔧 Installation

### 1. Arduino IDE Setup
```bash
# Install STM32 Core in Arduino IDE
# Go to Tools > Board > Board Manager
# Search for "STM32" and install "STM32 MCU based boards"
```

### 2. Library Installation
```bash
# Install required libraries via Arduino Library Manager
# or manually clone repositories to Arduino/libraries/
```

### 3. Clone Repository
```bash
git clone https://github.com/yourusername/lora-communication-system.git
cd lora-communication-system
```

## ⚙️ Configuration

### Transmitter Configuration

#### Sensor Selection
Edit `PayloadBuilder.cpp` to select your sensor:
```cpp
// Uncomment one of the following
//#define USE_HUMIDITY_4_20MA
//#define USE_SHT41
//#define USE_DS18B20
#define USE_MAX31865
```

#### Pin Configuration
```cpp
// Common pin definitions
#define LORA_NSS_PIN    10
#define LORA_DIO1_PIN   15
#define LORA_NRST_PIN   5
#define LORA_BUSY_PIN   4

// Sensor-specific pins
#define SENSOR_DATA_PIN 31  // For DS18B20
#define SENSOR_CS_PIN   19  // For MAX31865
#define HUMIDITY_ADC_PIN A1 // For 4-20mA sensor
```

#### Power Management
```cpp
// Configure sleep intervals
deepSleep.sleepMinutes(15);  // Sleep for 15 minutes
// or
deepSleep.sleepSeconds(30);  // Sleep for 30 seconds
```

#### ATPC Configuration
```cpp
// Initialize ATPC with custom parameters
ATPC atpc(&radio, 
         22,      // Initial power (dBm)
         -90.0,   // Target RSSI (dBm)
         -9,      // Minimum power (dBm)
         22);     // Maximum power (dBm)
```

### Receiver Configuration

#### FreeRTOS Task Configuration
```cpp
// Task priorities and stack sizes
#define RECEIVE_TASK_PRIORITY    2
#define PRINT_TASK_PRIORITY      1
#define WATCHDOG_TASK_PRIORITY   3

#define RECEIVE_TASK_STACK_SIZE  256
#define PRINT_TASK_STACK_SIZE    256
#define WATCHDOG_TASK_STACK_SIZE 128
```

## 🎯 Usage

### Transmitter Example
```cpp
#include "PayloadBuilder.h"
#include "LoRaTransmitter.h"
#include "ATPC.h"
#include "DeepSleep.h"
#include "MAX31865_Sensor.h"

// Initialize components
STM32WLx radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_NRST_PIN, LORA_BUSY_PIN);
ATPC atpc(&radio, 22, -90.0, -9, 22);
LoRaTransmitter transmitter(&radio, &atpc);
MAX31865_Sensor sensor(SENSOR_CS_PIN);
PayloadBuilder payloadBuilder(&sensor);
DeepSleep deepSleep(&radio, reinitializeSystem);

void setup() {
    Serial.begin(115200);
    
    // Initialize LoRa
    if (radio.begin() == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa initialized successfully!");
    }
    
    // Initialize sensor
    if (sensor.begin()) {
        Serial.println("Sensor initialized successfully!");
    }
    
    // Initialize deep sleep
    deepSleep.begin();
}

void loop() {
    float temperature, humidity;
    
    // Read sensor data
    if (sensor.readSensor(temperature, humidity)) {
        // Build payload
        uint8_t payload[PayloadBuilder::PAYLOAD_SIZE];
        payloadBuilder.buildPayload(payload, temperature, humidity);
        
        // Transmit with acknowledgment
        if (transmitter.transmitWithAck(payload, sizeof(payload))) {
            Serial.println("Transmission successful!");
        } else {
            Serial.println("Transmission failed!");
        }
    }
    
    // Enter deep sleep
    deepSleep.sleepMinutes(15);
}

void reinitializeSystem() {
    // Reinitialize components after wake-up
    radio.begin();
    sensor.reinitialize();
    Serial.begin(115200);
}
```

### Receiver Example
```cpp
#include "recieve_task.h"
#include "ackandcrc.h"

STM32WLx radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_NRST_PIN, LORA_BUSY_PIN);
QueueHandle_t packetQueue;

void setup() {
    Serial.begin(115200);
    
    // Initialize LoRa
    if (radio.begin() == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa receiver initialized!");
    }
    
    radio.setDio1Action(setFlag);
    radio.startReceive();
    
    // Create FreeRTOS queue
    packetQueue = xQueueCreate(10, sizeof(PacketMsg));
    
    // Create tasks
    xTaskCreate(ReceiveTask, "Receive", 256, NULL, 2, NULL);
    xTaskCreate(PrintTask, "Print", 256, NULL, 1, NULL);
    xTaskCreate(WatchdogTask, "Watchdog", 128, NULL, 3, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
}

void loop() {
    // FreeRTOS handles execution
}
```

## 📖 API Reference

### ATPC Class
```cpp
// Constructor
ATPC(STM32WLx* radioInstance, int initialTxPower, float targetRssi, int minPower, int maxPower);

// Methods
void adjustTransmitPower(float feedbackRssi);
int getCurrentTxPower() const;
void setTargetRssi(float targetRssi);
void setPowerLimits(int minPower, int maxPower);
void resetToInitialPower();
```

### DeepSleep Class
```cpp
// Constructor
DeepSleep(STM32WLx* radioPtr, void (*callback)());

// Methods
void sleep(uint32_t durationMs);
void sleepSeconds(uint32_t durationSeconds);
void sleepMinutes(uint32_t durationMinutes);
uint32_t getWakeupCounter() const;
bool getIsFirstRun() const;
```

### PayloadBuilder Class
```cpp
// Constructor (sensor-specific)
PayloadBuilder(SensorType* sensor);

// Methods
void buildPayload(uint8_t* payload, float temperature, float humidity);
uint32_t getUnixTimestamp();
float getBatteryPercentage();
float readBatteryVoltage();
```

### LoRaTransmitter Class
```cpp
// Constructor
LoRaTransmitter(STM32WLx* radioInstance, ATPC* atpcInstance);

// Methods
bool transmitWithAck(const uint8_t* payload, size_t payloadSize);
void setAckTimeout(int timeoutMs);
void setMaxRetries(int retries);
float getLastAckRssi() const;
```

## 📊 Payload Format

### Data Packet Structure (17 bytes)
```
Byte 0:    Header (0xAA)
Byte 1-4:  Device ID (32-bit)
Byte 5-6:  Temperature (16-bit, scaled by 100)
Byte 7-8:  Humidity (16-bit, scaled by 10)
Byte 9-10: Battery Voltage (16-bit, mV)
Byte 11-14: Timestamp (32-bit Unix)
Byte 15:   Checksum
Byte 16:   Footer (0xFF)
```

### ACK Packet Structure (7 bytes)
```
Byte 0:   ACK Header (0x55)
Byte 1-4: Device ID Echo
Byte 5-6: RSSI Value (16-bit, scaled by 10)
```

## 🔍 Troubleshooting

### Common Issues

#### Transmitter Not Waking Up
- Check deep sleep configuration
- Verify RTC clock source
- Ensure proper pin configuration for wake-up

#### Poor Transmission Range
- Check antenna connection
- Verify frequency settings
- Adjust ATPC target RSSI
- Check for interference

#### Sensor Reading Errors
- Verify sensor wiring
- Check power supply stability
- Ensure proper sensor initialization
- Review sensor-specific troubleshooting

#### High Power Consumption
- Verify deep sleep entry
- Check for active peripherals
- Review pin configuration
- Ensure radio sleep mode

### Debug Commands
```cpp
// Enable debug output
#define DEBUG_ENABLED 1

// ATPC debugging
Serial.print("Current TX Power: ");
Serial.println(atpc.getCurrentTxPower());

// Battery monitoring
Serial.print("Battery: ");
Serial.print(payloadBuilder.getBatteryPercentage());
Serial.println("%");

// Transmission debugging
Serial.print("Last ACK RSSI: ");
Serial.println(transmitter.getLastAckRssi());
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow existing code style
- Add comprehensive comments
- Include error handling
- Test with multiple sensor types
- Update documentation

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

For support and questions:
- Create an issue on GitHub
- Check the [Wiki](../../wiki) for additional documentation
- Review existing issues for similar problems

## 🔄 Version History

### v2.0.0 (Latest)
- Added multi-sensor support
- Implemented ATPC (Adaptive Transmit Power Control)
- Enhanced deep sleep management
- Added FreeRTOS receiver implementation
- Improved error handling and validation
- Battery monitoring enhancements

### v1.0.0
- Initial release
- Basic LoRa communication
- Simple sensor integration

## 🏗️ Architecture

```
Transmitter Side:
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Layer  │    │  Payload Layer  │    │   Radio Layer   │
│   - SHT41       │───▶│  - Builder      │───▶│  - LoRa TX      │
│   - DS18B20     │    │  - Validation   │    │  - ATPC         │
│   - MAX31865    │    │  - Formatting   │    │  - Retry Logic  │
│   - 4-20mA      │    └─────────────────┘    └─────────────────┘
└─────────────────┘             │                       │
         │                      ▼                       ▼
         │            ┌─────────────────┐    ┌─────────────────┐
         │            │  Power Mgmt     │    │  Deep Sleep     │
         └──────────▶│  - Battery Mon  │    │  - Wake/Sleep   │
                      │  - Efficiency   │    │  - Pin Config   │
                      └─────────────────┘    └─────────────────┘

Receiver Side:
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Radio Layer   │    │  Task Manager   │    │  Output Layer   │
│   - LoRa RX     │──▶│  - FreeRTOS     │──▶│  - Serial       │
│   - ACK TX      │    │  - Queues       │    │  - Processing   │
│   - Validation  │    │  - Scheduling   │    │  - Logging      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────┐    ┌─────────────────┐
│  Watchdog       │    │  Error Handler  │
│  - System Mon   │    │  - Recovery     │
│  - Reliability  │    │  - Validation   │
└─────────────────┘    └─────────────────┘
```

---
