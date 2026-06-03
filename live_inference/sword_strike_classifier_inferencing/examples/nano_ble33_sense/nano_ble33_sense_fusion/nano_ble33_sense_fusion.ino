/* Edge Impulse Ingestion SDK - Throttled 6-Axis Continuous Inference
   Hardware: Arduino Nano 33 BLE (BMI270 IMU Revision)
*/

#include <sword_strike_classifier_inferencing.h>
#include <Arduino_BMI270_BMM150.h> 

#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f

// Allocate a flat buffer for the sliding window capture frames
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

// --- Throttling Configuration ---
const unsigned long PREDICTION_INTERVAL_MS = 2000; // Change this to alter prediction speed (e.g., 2000 = 2 seconds)
unsigned long lastPredictionTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial); 
    
    Serial.println("--- 6-AXIS STANDALONE SWORD TRACKER BOOT ---");

    if (!IMU.begin()) {
        Serial.println("CRITICAL ERROR: BMI270 IMU failed to initialize!");
        while (1);
    }
    Serial.println("IMU hardware online.");

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) {
        Serial.print("ERROR: Model layout expects ");
        Serial.print(EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        Serial.println(" axes, but this script is hardcoded for 6.");
        while(1);
    }

    Serial.println("Pipeline active. Predictions throttled to every 2 seconds.");
    Serial.println("--------------------------------------------------\n");
    
    lastPredictionTime = millis();
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop() {
    // 1. Keep sampling data smoothly at 100 Hz to keep the moving window completely full
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

    numpy::roll(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -6);

    float ax, ay, az;
    float gx, gy, gz;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        for (int i = 0; i < 3; i++) {
            float* axis_ptr = (i == 0) ? &ax : ((i == 1) ? &ay : &az);
            if (fabs(*axis_ptr) > MAX_ACCEPTED_RANGE) {
                *axis_ptr = ei_get_sign(*axis_ptr) * MAX_ACCEPTED_RANGE;
            }
        }

        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 6] = ax * CONVERT_G_TO_MS2;
        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 5] = ay * CONVERT_G_TO_MS2;
        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 4] = az * CONVERT_G_TO_MS2;
        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = gx;
        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = gy;
        inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = gz;
    }

    // 2. TIMING THROTTLE: Only run classification math if our interval has elapsed
    unsigned long currentTime = millis();
    if (currentTime - lastPredictionTime >= PREDICTION_INTERVAL_MS) {
        lastPredictionTime = currentTime; // Reset the clock pointer

        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err == 0) {
            
            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            
            if (res == EI_IMPULSE_OK) {
                int highest_idx = 0;
                float highest_conf = 0.0f;

                for (size_t ix = 0; ix < 4; ix++) {
                    if (result.classification[ix].value > highest_conf) {
                        highest_conf = result.classification[ix].value;
                        highest_idx = ix;
                    }
                }

                // Print out clean, paced lines
                Serial.print("Paced Prediction: ");
                Serial.print(result.classification[highest_idx].label);
                Serial.print(" | Confidence: ");
                Serial.print(highest_conf * 100.0f, 1);
                Serial.println("%");
            }
        }
    }

    // Maintain the background 100 Hz tracking heartbeat 
    int64_t time_to_wait = next_tick - (int64_t)micros();
    if (time_to_wait > 0) {
        delayMicroseconds(time_to_wait);
    }
}