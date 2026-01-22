# RiaLineTracerR4

Arduino UNO R4 WiFi + Zumo Reflectance Sensor Array(RC) + (Zumo Shield style motor driver) 조합으로 **교육용 라인트레이서**를 만들기 위한 라이브러리입니다.

이 라이브러리는 초보자가 코드 흐름을 이해하기 쉽게 아래 과정을 **함수 단위로 분리**합니다.

1. 캘리브레이션 (Calibration)  
2. 라인 위치 읽기 (Read line position)  
3. 오차 계산 (Compute error)  
4. PID 보정값 계산 (Compute correction)  
5. 모터 구동 (Drive motors with PWM)

---

## Hardware Assumptions

### Sensor (RC type)
- Zumo Reflectance Sensor Array (RC 타이밍 방식)
- 센서 핀은 반드시 **LEFT → RIGHT 순서**로 배열에 넣습니다.

예:
```cpp
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 }; // LEFT -> RIGHT
```

### Emitter pin
- IR emitter 제어 핀은 기본값이 **2번**입니다.
- 다른 핀을 쓰면 생성자에서 변경하세요.

예:
```cpp
RiaLineTracerR4 tracer(sensorPins, 6, /*emitterPin=*/2);
```

### Motor (Zumo Shield style default)
기본 모터 핀(필요 시 변경 가능):
- leftDir = D8, leftPwm = D10
- rightDir = D7, rightPwm = D9

방향이 반대로 동작하면:
```cpp
tracer.setDirPolarity(false); // 또는 true로 복구
```

---

## Quick Start (교육용 권장 흐름)

수업에서는 examples를 **01 → 02 → 03** 순서로 진행하는 것을 권장합니다.

### Example 01: Raw 센서값 읽기 (기초)
- 목표: RC 타이밍 raw(us) 값이 바닥/라인에서 어떻게 변하는지 관찰
- 사용 함수: `begin()`, `readRaw()`

경로:
- `File > Examples > RiaLineTracerR4 > 01_read_raw`

---

### Example 02: 캘리브레이션 + 라인 위치 읽기 (중급)
- 목표: 캘리브레이션 후 `readLine()`이 position(0..5000)을 안정적으로 내는지 확인
- 사용 함수: `calibrateSpin()`, `readLine()`

경로:
- `File > Examples > RiaLineTracerR4 > 02_calibrate_and_read_line`

캘리브레이션 시 주의:
- 제자리 회전 중에 센서들이 **라인(검정)과 바닥(흰색)**을 모두 경험해야 합니다.
- 그렇지 않으면 min/max 범위가 좁아져 라인 추종 성능이 불안정해질 수 있습니다.

---

### Example 03: PID 라인트레이싱 (완성)
- 목표: `loop()`에서 아래 순서가 코드로 그대로 보이게 구성  
  1) 라인 읽기  
  2) 오차 계산  
  3) PID 보정값 계산  
  4) 모터 구동  

경로:
- `File > Examples > RiaLineTracerR4 > 03_line_tracing_pid`

---

## API Overview (핵심 함수)

### Calibration
- `calibrateSpin(...)` : 모터로 좌/우 회전하면서 자동 캘리브레이션(교육용 추천)
- `calibrateManual(...)` : 사람이 직접 움직이며 캘리브레이션

### Read
- `readRaw(raw, timeoutUs, chargeUs)` : RC raw(us) 읽기
- `readCalibrated(cal, ...)` : 0..1000 정규화
- `readLine(whiteLine, ...)` : 라인 위치(position) + lost 반환

### Control
- `computeError(position)` : center 기준 error 반환
- `computeCorrection(error)` : PID correction 계산
- `drive(baseSpeed, correction, maxSpeed)` : 좌우 혼합 + PWM 적용
- `stop()` : 정지

---

## Tuning Notes (수업에서 설명 포인트)

- `timeoutUs` : RC 방전 측정 최대 시간 (예: 2000us)
- `chargeUs` : RC 충전 시간 (예: 10us)
- `noiseThreshold` : 너무 작은 센서값(노이즈)을 무시하기 위한 임계값 (예: 140)

PID 기본값은 환경/속도/배터리 상태에 따라 달라질 수 있습니다.  
먼저 안정적으로 따라가는 값을 찾은 뒤 속도를 올리는 것이 좋습니다.

---

## License
원하시는 라이선스 문구를 여기에 넣으세요.
