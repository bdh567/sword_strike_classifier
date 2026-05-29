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

const unsigned long SAMPLE_INTERVAL_MS = 25; // 25ms = 40 Hz, stable & below BMI270 ODR
const int BATCH_SIZE = 10;                   // rows per BLE notification (~250ms/packet)
unsigned long nextSampleTime = 0;            // fixed grid (catch-up scheduling) for constant rate
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
    nextSampleTime = startTime;
    resetBatch();

    while (central.connected()) {
      // Fixed-cadence grid: fire whenever we've reached the next scheduled tick.
      // Signed compare handles millis() rollover; catch-up keeps the rate constant
      // even after a blocking BLE flush stalls the loop.
      if ((long)(millis() - nextSampleTime) >= 0) {
        float ax, ay, az;
        float gx, gy, gz;

        // Read latest values every tick (no availability gate) so every slot
        // yields a complete 7-field row at a steady 40 Hz.
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        // Timestamp from the schedule grid, not millis(), so spacing stays exact.
        unsigned long ts = nextSampleTime - startTime;

        // Build one complete row "t,ax,ay,az,gx,gy,gz".
        char row[64];
        int n = snprintf(row, sizeof(row), "%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
                         ts, ax, ay, az, gx, gy, gz);

        if (n > 0) {
          // Flush first if this row wouldn't fit, so a row is never split/truncated.
          if (batchLen + n + 2 >= (int)sizeof(batchBuf)) {
            streamChar.writeValue(batchBuf);
            resetBatch();
          }
          if (batchCount > 0) batchBuf[batchLen++] = ';';
          memcpy(batchBuf + batchLen, row, n);
          batchLen += n;
          batchBuf[batchLen] = '\0';
          batchCount++;

          if (batchCount >= BATCH_SIZE) {
            streamChar.writeValue(batchBuf);
            resetBatch();
          }
        }

        nextSampleTime += SAMPLE_INTERVAL_MS;
      }
    }
  }
}