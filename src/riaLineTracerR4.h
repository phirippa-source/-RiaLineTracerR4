#pragma once
#include <Arduino.h>

/*
  RiaLineTracerR4
  - Arduino UNO R4 WiFi + Zumo Reflectance Sensor Array (RC timing)
  - Optimized RC readRaw(): early-exit + pending mask
  - Calibration + normalized readings (0..1000)
  - readLineWithStatus() returns lineLost
  - PID helper (anti-windup)
  - Motor PWM drive as member functions (default Zumo Shield pins)
  - Default emitterPin = 2
*/

class RiaLineTracerR4 {
public:
  struct MotorCommand {
    int16_t left;     // -maxSpeed..+maxSpeed
    int16_t right;    // -maxSpeed..+maxSpeed
    int32_t position; // 0..(N-1)*1000
    int16_t error;    // position - center
    bool    lineLost; // true if line not detected
  };

  // emitterPin default is 2
  RiaLineTracerR4(const uint8_t *sensorPins, uint8_t numSensors, int8_t emitterPin = 2);

  void begin(bool emitterAlwaysOn = true);

  // Calibration
  void resetCalibration();
  void calibrate(uint16_t iterations = 200,
                 uint16_t timeoutMicros = 2500,
                 uint8_t chargeMicros = 10,
                 uint16_t interDelayMs = 5);

  // Sensor I/O
  uint16_t readRaw(uint16_t *rawValues,
                   uint16_t timeoutMicros = 2500,
                   uint8_t chargeMicros = 10);

  void readCalibrated(uint16_t *calibratedValues,
                      uint16_t timeoutMicros = 2500,
                      uint8_t chargeMicros = 10);

  int32_t readLine(bool whiteLine = false,
                   uint16_t timeoutMicros = 2500,
                   uint8_t chargeMicros = 10,
                   uint16_t noiseThreshold = 50);

  int32_t readLineWithStatus(bool &lineLost,
                             bool whiteLine = false,
                             uint16_t timeoutMicros = 2500,
                             uint8_t chargeMicros = 10,
                             uint16_t noiseThreshold = 50);

  // PID
  void setPID(float kp, float ki, float kd);
  void setIntegralLimit(float limitAbs);
  void resetPID();

  // Line-lost strategy
  void setHoldLastOnLost(bool enable);
  void setSearchOnLost(bool enable);
  void setSearchTurn(int16_t turn);

  // Motor PWM (member)
  void setMotorPins(uint8_t leftDirPin, uint8_t leftPwmPin,
                    uint8_t rightDirPin, uint8_t rightPwmPin);
  void setDirPolarity(bool lowIsForward);
  void setMinPwmWhenMoving(uint8_t minPwm);

  void applyMotor(int16_t left, int16_t right);

  MotorCommand step(int16_t baseSpeed,
                    int16_t maxSpeed,
                    bool whiteLine = false,
                    uint16_t timeoutMicros = 2500,
                    uint8_t chargeMicros = 10,
                    uint16_t noiseThreshold = 50);

  // Accessors
  uint8_t numSensors() const { return _numSensors; }
  bool calibrated() const { return _calibrated; }
  int32_t lastPosition() const { return _lastPosition; }
  int8_t emitterPin() const { return _emitterPin; }

private:
  static constexpr uint8_t MAX_SENSORS = 8;

  // sensor core
  uint8_t _numSensors;
  int8_t  _emitterPin;
  bool    _emitterAlwaysOn;

  uint8_t  _pins[MAX_SENSORS];
  bool     _calibrated;
  uint16_t _calibMin[MAX_SENSORS];
  uint16_t _calibMax[MAX_SENSORS];
  int32_t  _lastPosition;

  void setEmitter(bool on);
  uint16_t readRawInternal(uint16_t *values, uint16_t timeoutMicros, uint8_t chargeMicros);

  // PID core
  float _kp, _ki, _kd;
  float _integral;
  float _prevError;
  bool  _hasPrev;
  float _integralLimitAbs;

  float pidUpdate(float error, float dtSeconds);

  // step timing
  uint32_t _lastStepMicros;
  float    _lastCorrection;

  // lineLost strategy
  bool   _holdLastOnLost;
  bool   _searchOnLost;
  int16_t _searchTurn;

  // motor core
  uint8_t _pinDirL, _pinPwmL;
  uint8_t _pinDirR, _pinPwmR;
  bool    _dirLowIsForward;
  uint8_t _minPwmWhenMoving;

  void initMotorPins();
  void setOneMotor(uint8_t pinDir, uint8_t pinPwm, int16_t cmd);
  uint8_t clampPwmFromCommand(int16_t cmd) const;

  static int16_t clampI16(int32_t v, int16_t lo, int16_t hi);
};
