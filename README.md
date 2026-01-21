# RiaLineTracerR4

Stable line tracing library for **Arduino UNO R4 WiFi** + **Zumo Reflectance Sensor Array (RC timing type)**.

This library combines:
- Fast RC timing sensor read (early-exit)
- Calibration (min/max) + normalization (0..1000)
- Weighted line position (`readLine`, `readLineWithStatus`)
- Small PID helper + line-lost handling

> Target platform
- Arduino UNO R4 WiFi (architecture: renesas_uno)

---

## Folder Layout (Manual Install)

Place this library folder here:

- Windows: `C:\Users\<YOU>\Documents\Arduino\libraries\RiaLineTracerR4\`
- macOS: `~/Documents/Arduino/libraries/RiaLineTracerR4/`
- Linux: `~/Arduino/libraries/RiaLineTracerR4/`

Recommended structure:

RiaLineTracerR4/

├─ library.properties

├─ README.md

├─ src/

  ├─ riaLineTracerR4.h

  └─ riaLineTracerR4.cpp

└─ examples/

   └─ basic_line_trace/
   
      └─ basic_line_trace.ino

After copying, restart Arduino IDE.

---

## Wiring / Sensor Pins

This project assumes a **6-sensor RC timing array** connected to the following pins
(in the same order as your array mapping):

`{ 5, A2, A0, 11, A3, 4 }`

If you have an IR emitter control pin, pass it as `emitterPin`.  
If you do not use/know it, set `emitterPin = -1`.

---

## Quick Start Example
Create an example sketch (or use examples/basic_line_trace/basic_line_trace.ino):

```cpp
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
```
