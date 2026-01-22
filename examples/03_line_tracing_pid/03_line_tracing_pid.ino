#include <RiaLineTracerR4.h>

// Sensor pins (LEFT -> RIGHT)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

// ---------- Confirmed stable profile ----------
static constexpr float KP = 0.08f;
static constexpr float KI = 0.0f;
static constexpr float KD = 0.01f;

static constexpr int16_t BASE_SPEED = 170;
static constexpr int16_t MAX_SPEED  = 220;

static constexpr uint16_t TIMEOUT_US   = 2000;
static constexpr uint8_t  CHARGE_US    = 10;
static constexpr uint16_t NOISE_THRESH = 140;

static constexpr bool WHITE_LINE = false;

// ---------- Auto calibration (spin) ----------
static constexpr uint16_t CAL_LOOPS    = 400;
static constexpr int16_t  CAL_TURN_PWM = 130;
static constexpr uint16_t CAL_BLOCK    = 80;
static constexpr uint8_t  CAL_DELAY_MS = 10;
static constexpr uint16_t START_DELAY_MS = 1500;

void setup() {
  Serial.begin(115200);

  // 1) 초기화
  tracer.begin(true);

  delay(START_DELAY_MS);

  // 2) 캘리브레이션
  tracer.calibrateSpin(
    CAL_LOOPS,
    CAL_TURN_PWM,
    CAL_BLOCK,
    TIMEOUT_US,
    CHARGE_US,
    CAL_DELAY_MS
  );

  // 3) PID 설정
  tracer.setPID(KP, KI, KD);
  tracer.setIntegralLimit(1500.0f);
  tracer.resetPID();

  Serial.println("03_line_tracing_pid READY");
}

void loop() {
  // (1) 라인 읽기
  auto line = tracer.readLine(WHITE_LINE, TIMEOUT_US, CHARGE_US, NOISE_THRESH);

  // 라인 완전 이탈 시: 안전 정지(교육용 기본 정책)
  if (line.lost) {
    tracer.stop();
    return;
  }

  // (2) 오차 계산 (center 기준)
  int32_t error = tracer.computeError(line.position);

  // (3) PID로 보정값 계산
  int32_t correction = tracer.computeCorrection(error);

  // (4) 모터 구동(좌우 혼합 + PWM 적용)
  tracer.drive(BASE_SPEED, correction, MAX_SPEED);

  // 디버그(선택): 너무 자주 출력하면 제어가 흔들릴 수 있어 200ms 주기
  static uint32_t last = 0;
  if (Serial && (millis() - last >= 200)) {
    last = millis();
    Serial.print("pos="); Serial.print(line.position);
    Serial.print(" err="); Serial.print(error);
    Serial.print(" corr="); Serial.println(correction);
  }
}
