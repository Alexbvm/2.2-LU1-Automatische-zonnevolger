#include <ESP32Servo.h>

// --- Pin definities ---
#define LDR_Y 26
#define LDR_G 25
#define LDR_B 34
const int servoPin = 27;

// --- Servo ---
Servo solar;
float servoPos = 90;

// --- Sensor data struct en queue ---
struct SensorData { float Y, G, B; };
QueueHandle_t sensorQueue;

// --- Parameters ---
float ALPHA = 0.40f;           // smoothing factor
float EQUAL_THRESHOLD = 5.0f;  // % verschil threshold
float SERVO_SMOOTHING = 0.05f; // servo smoothing factor

// --- Sensor Task ---
void sensorTask(void *pvParameters) {
  float Y_smooth = analogRead(LDR_Y);
  float G_smooth = analogRead(LDR_G);
  float B_smooth = analogRead(LDR_B);

  for(;;) {
    // Lees RAW
    float Y_raw = analogRead(LDR_Y);
    float G_raw = analogRead(LDR_G);
    float B_raw = analogRead(LDR_B);

    // Exponential smoothing
    Y_smooth = ALPHA * Y_raw + (1.0f - ALPHA) * Y_smooth;
    G_smooth = ALPHA * G_raw + (1.0f - ALPHA) * G_smooth;
    B_smooth = (ALPHA * B_raw + (1.0f - ALPHA) * B_smooth) * 0.9; //0.9 voor de hardware compensatie
// Plaats data in queue
    SensorData data = { Y_smooth, G_smooth, B_smooth };
    if (sensorQueue != nullptr) {
      xQueueOverwrite(sensorQueue, &data);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Servo Task ---
void servoTask(void *pvParameters) {
  SensorData latest;

  for(;;) {
    if (sensorQueue != nullptr && xQueuePeek(sensorQueue, &latest, 0) == pdTRUE) {
      float Y = latest.Y;
      float G = latest.G;
      float B = latest.B;

      // Bereken target hoek
      float totalSmooth = Y + G + B;
      float target = 90.0f;

      if (totalSmooth > 0.1f) {
        // percentages
        float Y_pct = Y / totalSmooth * 100.0f;
        float G_pct = G / totalSmooth * 100.0f;
        float B_pct = B / totalSmooth * 100.0f;

        float maxPct = max(Y_pct, max(G_pct, B_pct));
        float minPct = min(Y_pct, min(G_pct, B_pct));
        float diffPct = maxPct - minPct;

        if (diffPct < EQUAL_THRESHOLD) {
          target = 90.0f;
        } else {
          float lowest   = min(Y, min(G, B));
          float Y_diff   = Y - lowest;
          float G_diff   = G - lowest;
          float B_diff   = B - lowest;
          float totalDiff = Y_diff + G_diff + B_diff;

          if (totalDiff > 0.01f) {
            target = (Y_diff * 180.0f +
                      G_diff *  90.0f +
                      B_diff *   0.0f) / totalDiff;
          } else {
            target = 90.0f;
          }
        }
      }

// Servo smooth beweging 
      servoPos += (target - servoPos) * SERVO_SMOOTHING;
      solar.write((int)servoPos);

      Serial.print("Y: "); Serial.print(Y, 1);
      Serial.print("  G: "); Serial.print(G, 1);
      Serial.print("  B: "); Serial.print(B, 1);
      Serial.print("  Target: "); Serial.print(target, 1);
      Serial.print("  Servo: "); Serial.println(servoPos, 1);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);

  solar.attach(servoPin, 500, 2400);
  solar.write((int)servoPos);

  sensorQueue = xQueueCreate(1, sizeof(SensorData));

  // Tasks aanmaken
  xTaskCreatePinnedToCore(
    sensorTask,        // functie
    "SensorTask",      // naam
    4096,              // stack size (in woorden, niet bytes)
    nullptr,           // parameters
    1,                 // prioriteit
    nullptr,           // task handle
    0                  // core 0
  );

  xTaskCreatePinnedToCore(
    servoTask,
    "ServoTask",
    4096,
    nullptr,
    2,                 // iets hogere prioriteit voor servo
    nullptr,
    1                  // core 1
  );
}

// --- Loop (niet nodig) ---
void loop() {
  // Hoeft nie, alles draait in FreeRTOS
}