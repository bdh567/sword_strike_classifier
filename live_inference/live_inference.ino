/* TinyML Sword Tracker - Maximum Serial Debugging Edition
  Hardware: Arduino Nano 33 BLE (BMI270 IMU Revision)
  
  DIAGNOSTICS: Open your Arduino IDE Serial Monitor at 115200 Baud to watch data flow.
*/

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// Matches your compiled Edge Impulse zip library header exactly
#include <sword_strike_classifier_inferencing.h>

#define SWORD_SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define PREDICTION_CHAR_UUID      "19b10001-e8f2-537e-4f6c-d104768a1214"

BLEService        swordService(SWORD_SERVICE_UUID);
BLECharacteristic predictionChar(PREDICTION_CHAR_UUID, BLERead | BLENotify, 32);

// --- Sliding Window Configuration ---
constexpr int WINDOW_SIZE = EI_CLASSIFIER_RAW_SAMPLE_COUNT; 
float inference_buffer[WINDOW_SIZE];

const unsigned long SAMPLE_INTERVAL_MS = 10; // 100 Hz strict time pacing
unsigned long lastSampleTime = 0;
unsigned long loopCounter = 0;

void setup() {
    Serial.begin(115200);
    delay(2000); // Give user time to open the Serial Monitor window
    
    Serial.println("\n=============================================");
    Serial.println("   SWORD TRACKER DIAGNOSTIC FIRMWARE BOOT   ");
    Serial.println("=============================================");
    
    Serial.print("Step 1: Initializing BMI270 IMU... ");
    if (!IMU.begin()) { 
        Serial.println("CRITICAL FAILURE! Hardware sensor missing or dead."); 
        while (1); 
    }
    Serial.println("SUCCESS.");
    
    Serial.print("Step 2: Initializing Bluetooth Low Energy... ");
    if (!BLE.begin()) { 
        Serial.println("CRITICAL FAILURE! BLE Radio failed to initialize."); 
        while (1); 
    }
    Serial.println("SUCCESS.");

    // Fill buffer with baseline zeros
    Serial.print("Step 3: Pre-allocating " + String(WINDOW_SIZE) + " float points to ring buffer... ");
    for (int i = 0; i < WINDOW_SIZE; i++) {
        inference_buffer[i] = 0.0f;
    }
    Serial.println("SUCCESS.");

    BLE.setLocalName("SmartSword-Inference");
    BLE.setAdvertisedService(swordService);
    swordService.addCharacteristic(predictionChar);
    BLE.addService(swordService);
    
    predictionChar.writeValue("idle");
    BLE.advertise();
    
    Serial.println("\n[SYSTEM READY] Open your browser app and connect over BLE.");
    Serial.println("-------------------------------------------------------------");
}

void loop() {
    BLEDevice central = BLE.central();
    
    if (central && central.connected()) {
        Serial.print("\n>>> Connection Established with Central Device: ");
        Serial.println(central.address());
        
        unsigned long lastThrottledPrint = 0;
        
        while (central.connected()) {
            unsigned long currentTime = millis();
            
            if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
                lastSampleTime = currentTime;
                loopCounter++;
                
                float ax, ay, az;
                float gx, gy, gz;

                // Test if hardware data registers are actually providing updates
                if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
                    IMU.readAcceleration(ax, ay, az);
                    IMU.readGyroscope(gx, gy, gz);

                    // --- Sliding Window Ring Buffer Management ---
                    for (int i = 0; i < WINDOW_SIZE - 6; i++) {
                        inference_buffer[i] = inference_buffer[i + 6];
                    }
                    
                    inference_buffer[WINDOW_SIZE - 6] = ax;
                    inference_buffer[WINDOW_SIZE - 5] = ay;
                    inference_buffer[WINDOW_SIZE - 4] = az;
                    inference_buffer[WINDOW_SIZE - 3] = gx;
                    inference_buffer[WINDOW_SIZE - 2] = gy;
                    inference_buffer[WINDOW_SIZE - 1] = gz;
                    
                    // --- Edge Impulse C++ Signal Conversion Check ---
                    signal_t signal;
                    int err = numpy::signal_from_buffer(inference_buffer, WINDOW_SIZE, &signal);
                    if (err != 0) {
                        Serial.println("⚠️ SDK Error: Failed to generate math signal array pointer.");
                        continue;
                    }

                    // --- Execute 1D CNN Local Classification Math ---
                    ei_impulse_result_t result = { 0 };
                    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
                    if (res != EI_IMPULSE_OK) {
                        Serial.println("⚠️ SDK Error: Inference engine failed execution matrix computation.");
                        continue;
                    }

                    // --- Extract Maximum Confidence Value (Argmax) ---
                    int highest_index = 0;
                    float highest_confidence = 0.0;
                    
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        if (result.classification[ix].value > highest_confidence) {
                            highest_confidence = result.classification[ix].value;
                            highest_index = ix;
                        }
                    }

                    // --- HIGH-SPEED MONITORING PRINTOUT (Throttled to 300ms to stay readable) ---
                    if (currentTime - lastThrottledPrint >= 300) {
                        lastThrottledPrint = currentTime;
                        
                        Serial.println("\n--- [HEARTBEAT LOG] ---");
                        Serial.print("Raw Sensor Framework -> Accel: ["); 
                        Serial.print(ax, 2); Serial.print(", "); Serial.print(ay, 2); Serial.print(", "); Serial.print(az, 2);
                        Serial.print("] | Gyro: [");
                        Serial.print(gx, 1); Serial.print(", "); Serial.print(gy, 1); Serial.print(", "); Serial.print(gz, 1);
                        Serial.println("]");
                        
                        Serial.println("Model Live Probability Output Array:");
                        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                            Serial.print("  ├── Class [");
                            Serial.print(result.classification[ix].label);
                            Serial.print("]: ");
                            Serial.print(result.classification[ix].value * 100.0f, 1);
                            Serial.println("%");
                        }
                        Serial.print("  └── WINNING TARGET: ");
                        Serial.print(result.classification[highest_index].label);
                        Serial.print(" ("); Serial.print(highest_confidence * 100.0f, 1); Serial.println("%)");
                    }

                    // --- Active Strike Over-The-Air Broadcast ---
                    if (highest_confidence > 0.85) {
                        String predictedLabel = String(result.classification[highest_index].label);
                        
                        if (predictedLabel == "vertical" || predictedLabel == "horizontal") {
                            
                            // Attempt a native write over Bluetooth radio infrastructure
                            bool txSuccess = predictionChar.writeValue((uint8_t*)predictedLabel.c_str(), predictedLabel.length());
                            
                            String upperLabel = predictedLabel;
                            upperLabel.toUpperCase();
                            
                            Serial.println("\n⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐");
                            Serial.print("🔥 STRIKE THRESHOLD EXCEEDED! Class: ");
                            Serial.println(upperLabel);
                            Serial.print("Confidence: "); Serial.print(highest_confidence * 100.0f, 2); Serial.println("%");
                            Serial.print("BLE Notification Transmit Status: ");
                            Serial.println(txSuccess ? "SUCCESS (Dispatched)" : "FAILED (Buffer locked)");
                            Serial.println("⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐\n");
                            
                            delay(400); // Dynamic frame lockout window to clear mechanical physical deceleration arcs
                            lastSampleTime = millis();
                        }
                    }
                } else {
                    // This warns you if the IMU registers aren't updating at 100Hz
                    if (currentTime - lastThrottledPrint >= 300) {
                        lastThrottledPrint = currentTime;
                        Serial.println("⚠️ Hardware Warning: IMU registers stalled. No data available.");
                    }
                }
            }
        }
        Serial.println("\n>>> Connection Dropped by Central Host Dashboard.");
        Serial.println("-------------------------------------------------------------");
    }
}