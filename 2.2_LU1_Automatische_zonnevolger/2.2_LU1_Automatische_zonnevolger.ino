#include <ESP32Servo.h>

#define LDR_Y 26
#define LDR_G 25
#define LDR_B 34

Servo solar;
int servoPin = 27;
float solarPos = 90;
float target;

float Y_raw, G_raw, B_raw;
float Y_smooth, G_smooth, B_smooth;
float Y_diff, G_diff, B_diff;
float Y_pct, G_pct, B_pct;
float lowestValue, totalValueSmooth, maxPct, minPct, diff, totalValueDiff;

float ALPHA = 0.40;
float EQUAL_THRESHOLD = 5.0;
float SERVO_SMOOTHING = 0.05;

void setup() {
  Serial.begin(115200);
  solar.attach(servoPin, 500, 2400);
  Y_smooth = analogRead(LDR_Y);
  G_smooth = analogRead(LDR_G);
  B_smooth = analogRead(LDR_B);
}

void loop() {
  // Read raw Y, G & B values
  Y_raw = analogRead(LDR_Y);
  G_raw = analogRead(LDR_G);
  B_raw = analogRead(LDR_B);

  // Smooth Y, G & B values
  Y_smooth = ALPHA * Y_raw + (1 - ALPHA) * Y_smooth;
  G_smooth = ALPHA * G_raw + (1 - ALPHA) * G_smooth;
  B_smooth = (ALPHA * B_raw + (1 - ALPHA) * B_smooth) * 0.9;

  // Calculate the % of the Y, G & B values
  totalValueSmooth = Y_smooth + G_smooth + B_smooth;
  Y_pct = Y_smooth / totalValueSmooth * 100;
  G_pct = G_smooth / totalValueSmooth * 100;
  B_pct = B_smooth / totalValueSmooth * 100;

  // Check if % values are (almost) the same
  maxPct = max(Y_pct, max(G_pct, B_pct));
  minPct = min(Y_pct, min(G_pct, B_pct));
  diff = maxPct - minPct;

  if(diff < EQUAL_THRESHOLD) {
    target = 90;
  } else {
    // Get lowest Y, G & B value
    lowestValue = min(Y_smooth, min(G_smooth, B_smooth));

    // Calculate diffrenct by lowest value
    Y_diff = Y_smooth - lowestValue;
    G_diff = G_smooth - lowestValue;
    B_diff = B_smooth - lowestValue;

    totalValueDiff = Y_diff + G_diff + B_diff;
    target = (Y_diff * 180 + G_diff * 90 + B_diff * 0) / totalValueDiff;
  }

  solarPos = solarPos + (target - solarPos) * SERVO_SMOOTHING;
  solar.write(solarPos);

  Serial.print(Y_smooth); Serial.print(" ");
  Serial.print(G_smooth); Serial.print(" ");
  Serial.print(B_smooth); Serial.print(" ");
  Serial.println();

  delay(50);
}