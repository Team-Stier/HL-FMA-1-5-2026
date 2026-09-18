#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cmath>
constexpr uint8_t A0=14, A1=15, A2=16, A3=17, A4=18;
constexpr uint8_t INPUT=0, OUTPUT=1, INPUT_PULLUP=2, LOW=0, HIGH=1, CHANGE=2;
extern uint32_t testMillis;
extern int testPwm[32];
extern uint8_t PINC, PCMSK1;
inline uint32_t millis() { return testMillis; }
inline uint32_t micros() { return testMillis*1000U; }
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline int analogRead(uint8_t) { return 512; }
inline void analogWrite(uint8_t pin, int pwm) { testPwm[pin]=pwm; }
inline void attachInterrupt(uint8_t, void(*)(), int) {}
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin-2; }
inline unsigned long pulseIn(uint8_t, uint8_t, unsigned long) { return 1500; }
class ArduinoHardware {};
