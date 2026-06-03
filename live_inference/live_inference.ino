#include "TensorFlowLite.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// 1. Update this to match your single .cc model file name
extern unsigned char sword_model_tflite[];
extern unsigned int sword_model_tflite_len;

// 2. Updated to your 4 exact classes
const char* label_map[4] = {
  "idle",
  "vertical",
  "horizontal",
  "diagonal"
};

// 3. Match your 1-second window size (100 Hz sampling for 1 second = 100 samples)
const int kWindowSize = 100;
const int kFeatureCount = 6;
const int kInputSize = kWindowSize * kFeatureCount;
const int kNumClasses = 4;

// 4. Set timing back to 100 Hz (10ms intervals) to match your 1-second clips
const unsigned long kSampleIntervalMs = 10; 

// Conversion factors
const float kGToMs2 = 9.80665f;
const float kDegToRad = 0.017453292519943295f;

// Memory Configuration
const int kTensorArenaSize = 56 * 1024;
__attribute__((aligned(32), section(".noinit"))) static uint8_t tensor_arena[kTensorArenaSize];

__attribute__((aligned(32), section(".noinit"))) static float window_buffer[kWindowSize][kFeatureCount];
static float model_input[kInputSize];
static float model_out[kNumClasses];

tflite::MicroErrorReporter micro_error_reporter;
tflite::ErrorReporter* error_reporter = &micro_error_reporter;
tflite::AllOpsResolver resolver;

int sample_index = 0;
bool have_accel = false;
bool have_gyro = false;
float latest_ax = 0.0f, latest_ay = 0.0f, latest_az = 0.0f;
float latest_gx = 0.0f, latest_gy = 0.0f, latest_gz = 0.0f;
unsigned long last_sample_time = 0;

int8_t QuantizeToInt8(float value, float scale, int zero_point) {
  int quantized = (int)lroundf(value / scale) + zero_point;
  if (quantized > 127) quantized = 127;
  if (quantized < -128) quantized = -128;
  return (int8_t)quantized;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("System Active. Free-running 4-class model...");
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    if (IMU.readAcceleration(ax, ay, az)) {
      latest_ax = ax * kGToMs2;
      latest_ay = ay * kGToMs2;
      latest_az = az * kGToMs2;
      have_accel = true;
    }
  }

  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;
    if (IMU.readGyroscope(gx, gy, gz)) {
      latest_gx = gx * kDegToRad;
      latest_gy = gy * kDegToRad;
      latest_gz = gz * kDegToRad;
      have_gyro = true;
    }
  }

  if (!have_accel || !have_gyro) return;
  if (millis() - last_sample_time < kSampleIntervalMs) return;
  last_sample_time = millis();

  // Populate sliding window array
  window_buffer[sample_index][0] = latest_ax;
  window_buffer[sample_index][1] = latest_ay;
  window_buffer[sample_index][2] = latest_az;
  window_buffer[sample_index][3] = latest_gx;
  window_buffer[sample_index][4] = latest_gy;
  window_buffer[sample_index][5] = latest_gz;

  sample_index++;

  if (sample_index >= kWindowSize) {
    sample_index = 0; // Reset index for next window
    
    // Flatten buffer into a 1D model input array (Assuming raw inputs without scaling)
    for (int i = 0; i < kWindowSize; i++) {
      for (int j = 0; j < kFeatureCount; j++) {
        model_input[i * kFeatureCount + j] = window_buffer[i][j];
      }
    }

    // Run TFLite Micro Inference Pipeline
    const tflite::Model* model = tflite::GetModel(sword_model_tflite);
    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter.AllocateTensors();

    TfLiteTensor* input = interpreter.input(0);
    TfLiteTensor* output = interpreter.output(0);

    // Quantize input on the fly
    for (int i = 0; i < kInputSize; i++) {
      input->data.int8[i] = QuantizeToInt8(model_input[i], input->params.scale, input->params.zero_point);
    }

    interpreter.Invoke();

    // Dequantize output back to probabilities
    int best_class = 0;
    float best_score = -100.0f;
    
    for (int i = 0; i < kNumClasses; i++) {
      model_out[i] = ((float)output->data.int8[i] - (float)output->params.zero_point) * output->params.scale;
      if (model_out[i] > best_score) {
        best_score = model_out[i];
        best_class = i;
      }
    }

    // Print to Serial Monitor
    Serial.print("Prediction: ");
    Serial.print(label_map[best_class]);
    Serial.print(" | Confidence: ");
    Serial.print(best_score * 100.0f, 1);
    Serial.println("%");
  }
}