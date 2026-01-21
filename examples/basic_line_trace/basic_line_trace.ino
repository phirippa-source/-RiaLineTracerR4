#include "riaLineTracerR4.h"

// Sensor pins in your specified order:
const uint8_t sensorPins[] = { 5, A2, A0, 11, A3, 4 };

// emitterPin: set to -1 if not used/unknown
RiaLineTracerR4 tracer(sensorPins, 6, -1);

// TODO: implement these for your motor driver (ZumoShield, TB6612, etc.)
void applyMotor(int16_t left, int16_t right) {
  // Example placeholder:
  // left/right ranges are typically -maxSpeed..+maxSpeed.
  // Convert to PWM + direction here.
}

void setup() {
  Serial.begin(115200);

  // emitterAlwaysOn=true: IR emitter stays ON
  tracer.begin(true);

  // Calibration:
  // Move the robot over BOTH line and background (shake left-right) during this phase.
  tracer.calibrate(
    200,   // iterations
    2500,  // timeoutMicros (typical 2000~3000us)
    10,    // chargeMicros
    5      // interDelayMs
  );

  // PID initial values (you MUST tune for your robot)
  tracer.pid().setGains(0.18f, 0.0f, 1.2f);
  tracer.pid().setIntegralLimit(1500.0f);

  // Line-lost strategy (choose one)
  tracer.setSearchOnLost(true);  // 적극적 재탐색
  tracer.setSearchTurn(90);      // 탐색 회전 세기(양수 기준)
  // tracer.setHoldLastOnLost(true); // 대신 "마지막 보정 유지" 전략을 쓰고 싶으면 searchOff + holdOn
}

void loop() {
  // baseSpeed: nominal forward speed
  // maxSpeed: clamp output
  auto cmd = tracer.step(
    140,   // baseSpeed
    220,   // maxSpeed
    false, // whiteLine (false: black line on white background)
    2500,  // timeoutMicros
    10,    // chargeMicros
    50     // noiseThreshold
  );

  applyMotor(cmd.left, cmd.right);

  // Debug print (rate-limited)
  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();
    Serial.print("lost="); Serial.print(cmd.lineLost);
    Serial.print(" pos="); Serial.print(cmd.position);
    Serial.print(" err="); Serial.print(cmd.error);
    Serial.print(" L="); Serial.print(cmd.left);
    Serial.print(" R="); Serial.println(cmd.right);
  }
}
