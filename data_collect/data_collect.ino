/* TinyML Sword Project - Continuous Bluetooth Data Streamer
   Hardware: Arduino Nano 33 BLE Sense (BMI270 IMU Variant)
*/

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

#define STREAM_SERVICE_UUID      "19b10000-e8f2-537e-4f6c-d104768a1214"
#define STREAM_CHAR_UUID         "19b10001-e8f2-537e-4f6c-d104768a1214"

BLEService        streamService(STREAM_SERVICE_UUID);
// Batch several CSV rows per notification so we keep up with 100 Hz over BLE.
// Worst-case row ~52 chars ("t,ax,ay,az,gx,gy,gz"); 10 rows + separators fits in 512.
BLEStringCharacteristic streamChar(STREAM_CHAR_UUID, BLERead | BLENotify, 512);

const unsigned long SAMPLE_INTERVAL_MS = 10; // 10ms = 100 Hz sampling rate
const int BATCH_SIZE = 10;                   // rows per BLE notification (~100ms/packet)
unsigned long lastSampleTime = 0;
unsigned long startTime = 0;                 // capture base so first row starts near 0

char batchBuf[512];
int  batchLen = 0;                           // bytes used in batchBuf
int  batchCount = 0;                         // rows buffered

void resetBatch() {
  batchLen = 0;
  batchCount = 0;
  batchBuf[0] = '\0';
}

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
    // Fresh recording: zero the clock and drop any stale buffered rows.
    startTime = millis();
    lastSampleTime = startTime;
    resetBatch();

    while (central.connected()) {
      unsigned long currentTime = millis();

      // Strict 100 Hz pacing loop
      if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        float ax, ay, az;
        float gx, gy, gz;

        // Only consume a slot once both sensors actually have a sample, so a
        // not-yet-ready IMU read doesn't silently drop the 10ms slot.
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          IMU.readAcceleration(ax, ay, az);
          IMU.readGyroscope(gx, gy, gz);
          lastSampleTime = currentTime;

          // Timestamp at read time so batching never distorts intra-batch spacing.
          unsigned long ts = currentTime - startTime;

          // Append one row "t,ax,ay,az,gx,gy,gz" (rows joined by ';') to the batch.
          char row[64];
          int n = snprintf(row, sizeof(row), "%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
                           ts, ax, ay, az, gx, gy, gz);
          if (n > 0 && batchLen + n + 1 < (int)sizeof(batchBuf)) {
            if (batchCount > 0) batchBuf[batchLen++] = ';';
            memcpy(batchBuf + batchLen, row, n);
            batchLen += n;
            batchBuf[batchLen] = '\0';
            batchCount++;
          }

          if (batchCount >= BATCH_SIZE) {
            streamChar.writeValue(batchBuf);
            resetBatch();
          }
        }
      }
    }
  }
}