#pragma once
#include <Arduino.h>

class RiaLineTracerR4 {
public:
  static constexpr uint8_t MAX_SENSORS = 8;

  struct StepCommand {
    bool    lineLost;
    int32_t position;   // 0..(N-1)*1000
    int32_t error;      // position - center
    int16_t left;       // signed motor command
    int16_t right;      // signed motor command
  };

public:
  // pins: sensor pins in LEFT->RIGHT order
  // numSensors: number of sensors (<= MAX_SENSORS)
  // emitterPin: IR emitter control pin (default 2)
  explicit RiaLineTracerR4(const uint8_t* pins, uint8_t numSensors, int8_t emitterPin = 2);

  // emitterAlwaysOn=true: keep emitter HIGH always after begin()
  void begin(bool emitterAlwaysOn = true);

  // -------------------- Sensor / Calibration --------------------
  void resetCalibration();

  // Read raw RC discharge times (microseconds) into values[]
  void readRaw(uint16_t* values,
               uint16_t timeoutMicros = 2000,
               uint8_t  chargeMicros  = 10);

  // Read calibrated values into values[] (0..1000)
  void readCalibrated(uint16_t* values,
                      uint16_t timeoutMicros = 2000,
                      uint8_t  chargeMicros  = 10);

  // Accumulate calibration data (min/max per sensor)
  void calibrate(uint16_t iterations = 200,
                 uint16_t timeoutMicros = 2000,
                 uint8_t  chargeMicros  = 10,
                 uint16_t interDelayMs  = 5);

  // Line position only
  int32_t readLine(bool whiteLine = false,
                   uint16_t timeoutMicros = 2000,
                   uint8_t  chargeMicros  = 10,
                   uint16_t noiseThreshold = 140);

  // Line position + lost flag (recommended)
  int32_t readLineWithStatus(bool& lineLost,
                            bool whiteLine = false,
                            uint16_t timeoutMicros = 2000,
                            uint8_t  chargeMicros  = 10,
                            uint16_t noiseThreshold = 140);

  // Auto calibration by spinning in-place (moves motors internally)
  void autoCalibrateSpin(
      uint16_t loops = 400,
      int16_t  turnPwm = 130,
      uint16_t block = 80,
      uint16_t timeoutMicros = 2000,
      uint8_t  chargeMicros = 10,
      uint8_t  perLoopDelayMs = 10
  );

  // -------------------- PID --------------------
  void setPID(float kp, float ki, float kd);
  void resetPID();
  void setIntegralLimit(float limitAbs);

  // -------------------- Motor --------------------
  // Default pins assume Zumo Shield style mapping:
  // leftDir=D8, leftPwm=D10, rightDir=D7, rightPwm=D9
  void setMotorPins(uint8_t leftDir, uint8_t leftPwm, uint8_t rightDir, uint8_t rightPwm);

  // lowIsForward=true: DIR LOW=forward, HIGH=reverse
  void setDirPolarity(bool lowIsForward);

  // if abs(speed)>0, ensure pwm >= minPwm (helps overcome motor deadzone)
  void setMinPwmWhenMoving(uint8_t minPwm);

  // Apply signed motor commands (-255..255 typical). Internally uses PWM + DIR.
  void applyMotor(int16_t left, int16_t right);

  // -------------------- Lost-line behavior --------------------
  void setSearchOnLost(bool enable);
  void setHoldLastOnLost(bool enable);
  void setSearchTurn(int16_t turnPwm);

  // High-level control loop: read line -> PID -> mix -> applyMotor
  StepCommand step(int16_t baseSpeed,
                   int16_t maxSpeed,
                   bool whiteLine = false,
                   uint16_t timeoutMicros = 2000,
                   uint8_t  chargeMicros  = 10,
                   uint16_t noiseThreshold = 140);

private:
  // helpers
  void ensureEmitterOnForRead_();
  void emitterOn_();
  void emitterOff_();

  int16_t clampMotor_(int32_t v, int16_t maxAbs) const;
  uint8_t clampPwmAbs_(int32_t v) const;
  int32_t roundToInt_(float x) const;

private:
  // Sensors
  uint8_t _numSensors;
  uint8_t _pins[MAX_SENSORS];

  // Emitter
  int8_t _emitterPin;
  bool   _emitterAlwaysOn;

  // Calibration
  bool     _calibrated;
  uint16_t _calibMin[MAX_SENSORS];
  uint16_t _calibMax[MAX_SENSORS];

  // Line tracking
  int32_t  _lastPosition;    // 0..(N-1)*1000
  int32_t  _lastError;       // position-center

  // PID state
  float    _kp, _ki, _kd;
  float    _integral;
  float    _integralLimitAbs;
  float    _lastDerivInput;  // last error for derivative
  uint32_t _lastPidMicros;

  // Motor pins
  uint8_t _leftDirPin;
  uint8_t _leftPwmPin;
  uint8_t _rightDirPin;
  uint8_t _rightPwmPin;

  bool    _dirLowIsForward;
  uint8_t _minPwmWhenMoving;

  // Lost-line policy
  bool    _searchOnLost;
  bool    _holdLastOnLost;
  int16_t _searchTurnPwm;
};
