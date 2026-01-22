#include <RiaLineTracerR4.h>

// Sensor pins (LEFT -> RIGHT)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

void setup() {
  Serial.begin(115200);
  tracer.begin(true);
  // 캘리브레이션: (기본값 사용)
  // calibrateSpin(loops=400, turnPwm=130, block=80, timeoutUs=2000, chargeUs=10, perLoopDelayMs=10)
  tracer.calibrateSpin();
}

void loop() {
  // 라인 위치 읽기: (기본값 사용)
  // readLine(whiteLine=false, timeoutUs=2000, chargeUs=10, noiseThreshold=140)
  auto line = tracer.readLine();

  // 출력(200ms에 1번)
  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    Serial.print("lost="); Serial.print(line.lost);
    Serial.print(" pos="); Serial.println(line.position); // 0..5000 (6개 센서면 0..5000)
  }
}
