/* TinyML Sword Project - Continuous Bluetooth Data Streamer
   Hardware: Arduino Nano 33 BLE Sense (BMI270 IMU Variant)
*/

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

#define STREAM_SERVICE_UUID      "19b10000-e8f2-537e-4f6c-d104768a1214"
#define STREAM_CHAR_UUID         "19b10001-e8f2-537e-4f6c-d104768a1214"

BLEService        streamService(STREAM_SERVICE_UUID);
// Allocate a 40-character string characteristic to hold a full CSV data row
BLEStringCharacteristic streamChar(STREAM_CHAR_UUID, BLERead | BLENotify, 40);

const unsigned long SAMPLE_INTERVAL_MS = 10; // 10ms = 100 Hz sampling rate
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  
  if (!IMU.begin()) { Serial.println("IMU Fail"); while (1); }
  if (!BLE.begin()) { Serial.println("BLE Fail"); while (1); }

  BLE.setLocalName("Sword-Data-Streamer");
  BLE.setAdvertisedService(streamService);
  streamService.addCharacteristic(streamChar);
  BLE.addService(streamService);
  BLE.advertise();

  Serial.println("Bluetooth streaming active. Connect via web dashboard...");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central && central.connected()) {
    while (central.connected()) {
      unsigned long currentTime = millis();
      
      // Strict 100 Hz pacing loop
      if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = currentTime;
        
        float ax, ay, az;
        float gx, gy, gz;

        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          IMU.readAcceleration(ax, ay, az);
          IMU.readGyroscope(gx, gy, gz);

          // Pack the 6-axis data into a highly compressed comma-separated string
          // Formatting: ax,ay,az,gx,gy,gz
          String dataRow = String(ax,2) + "," + String(ay,2) + "," + String(az,2) + "," +
                           String(gx,1) + "," + String(gy,1) + "," + String(gz,1);

          // Update characteristic to push wirelessly to the browser
          streamChar.writeValue(dataRow);
        }
      }
    }
  }
}