#include <RiaLineTracerR4.h>

// Sensor pins (LEFT -> RIGHT)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

// Calibration + read parameters (수업에서 숫자 의미 설명 포인트)
static constexpr uint16_t TIMEOUT_US   = 2000;
static constexpr uint8_t  CHARGE_US    = 10;
static constexpr uint16_t NOISE_THRESH = 140;
static constexpr bool     WHITE_LINE   = false;

// Auto calibration (spin)
static constexpr uint16_t CAL_LOOPS    = 400;
static constexpr int16_t  CAL_TURN_PWM = 130;
static constexpr uint16_t CAL_BLOCK    = 80;
static constexpr uint8_t  CAL_DELAY_MS = 10;

// 시작 전 대기(학생이 트랙 위에 올려놓을 시간)
static constexpr uint16_t START_DELAY_MS = 1500;

void setup() {
  Serial.begin(115200);

  // 1) 초기화
  tracer.begin(true);

  delay(START_DELAY_MS);

  // 2) 캘리브레이션: 제자리 좌/우 회전하면서 라인/바닥을 모두 경험하도록
  tracer.calibrateSpin(
    CAL_LOOPS,
    CAL_TURN_PWM,
    CAL_BLOCK,
    TIMEOUT_US,
    CHARGE_US,
    CAL_DELAY_MS
  );

  Serial.println("02_calibrate_and_read_line READY");
}

void loop() {
  // 3) 라인 위치 읽기
  auto line = tracer.readLine(WHITE_LINE, TIMEOUT_US, CHARGE_US, NOISE_THRESH);

  // 4) 출력(200ms에 1번)
  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    Serial.print("lost="); Serial.print(line.lost);
    Serial.print(" pos="); Serial.println(line.position); // 0..5000 (6개 센서면 0..5000)
  }
}
