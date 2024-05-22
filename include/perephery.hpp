#pragma once

#include <Arduino.h>

class TB6612Driver {
public:
    TB6612Driver(int motorRight, int motor_right_a_pin, int motor_right_b_pin, int motorLeft, int motor_left_a_pin, int motor_left_b_pin):
            _motorRight(motorRight),
            _motorRightAPin(motor_right_a_pin), 
            _motorRightBPin(motor_right_b_pin),
            _motorLeft(motorLeft),
            _motorLeftAPin(motor_left_a_pin), 
            _motorLeftBPin(motor_left_b_pin)
            {}
    void init( void ){
      pinMode(_motorRightAPin, OUTPUT);
      pinMode(_motorRightBPin, OUTPUT);
      pinMode(_motorLeftAPin, OUTPUT);
      pinMode(_motorLeftBPin, OUTPUT);
    }

    void set_power(int16_t left, int16_t right) {
        left = constrain(left, -255, 255);
        right = constrain(right, -255, 255);

        analogWrite(_motorRight, abs(right));
        analogWrite(_motorLeft, abs(left));

        if (right >= 0) {
            digitalWrite(_motorRightAPin, LOW);
            digitalWrite(_motorRightBPin, HIGH);
        } else {
            digitalWrite(_motorRightAPin, HIGH);
            digitalWrite(_motorRightBPin, LOW);
        }

        if (left >= 0) {
            digitalWrite(_motorLeftAPin, HIGH);
            digitalWrite(_motorLeftBPin, LOW);
        } else {
            digitalWrite(_motorLeftAPin, LOW);
            digitalWrite(_motorLeftBPin, HIGH);
        }
    }
    
private:
    int _motorRight;
    int _motorRightAPin;
    int _motorRightBPin;
    int _motorLeft;
    int _motorLeftAPin;
    int _motorLeftBPin;
};

class IRSensor {
public:
    IRSensor(int sensor_pin) : _sensorPin(sensor_pin) {}
    
    void init() {
        pinMode(_sensorPin, INPUT);
    }
    
    bool is_white() {
        return digitalRead(_sensorPin) == HIGH;
    }
    bool is_black() {
        return digitalRead(_sensorPin) == LOW;
    }
private:
    int _sensorPin;
};

typedef union {
    struct{
        bool LL:1;
        bool LC:1;
        bool CC:1;
        bool RC:1;
        bool RR:1;
    } __attribute__((packed));
    uint8_t all;
} line_sensors_t;

class IRSensorArray {
public:
  enum sensor_num_e{
    RR = 0, // Right Right
    RC, // Right Center
    CC, // Center Center
    LC, // Left Center
    LL, // Left Left
  };
    IRSensorArray(int rr_pin, int rc_pin, int cc_pin, int lc_pin, int ll_pin)
        : _sensors{ IRSensor(rr_pin), IRSensor(rc_pin), IRSensor(cc_pin), IRSensor(lc_pin), IRSensor(ll_pin) } {}

    void init() {
        for (int i = 0; i < 5; ++i) {
            _sensors[i].init();
        }
    }

    line_sensors_t get_line(void) {
      line_sensors_t line;
      line.all = 0x00;
      line.RR = _sensors[RR].is_black();
      line.RC = _sensors[RC].is_black();
      line.CC = _sensors[CC].is_black();
      line.LC = _sensors[LC].is_black();
      line.LL = _sensors[LL].is_black();
      return line;
    }

    void debug_print(void){
      line_sensors_t line = get_line();
      for(uint8_t i = 0; i < 5; i++){
        if(line.all & (1 << i)){
          Serial.print('1');
        }else{
          Serial.print('0');
        }
        if(i != 4){
          Serial.print('\t');
        }
      }
      Serial.print("\r\n");
    }

private:
    IRSensor _sensors[5];
};

typedef void (*ISRFunction)();

class Tachometer {
public:
    Tachometer(int encoder_pin, float wheel_circumference, int steps_per_revolution)
        : _encoderPin(encoder_pin), _wheelCircumference(wheel_circumference), _stepsPerRevolution(steps_per_revolution), _encoderCount(0) {}

    void init(ISRFunction isr) {
        pinMode(_encoderPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(_encoderPin), isr, CHANGE);
    }

    float get_distance() {
        return (_encoderCount / static_cast<float>(_stepsPerRevolution)) * _wheelCircumference;
    }

    void set_zero() {
        _encoderCount = 0;
    }

    void increment_ISR( void ){
      _encoderCount++;
    }

private:
    int _encoderPin;
    float _wheelCircumference;
    int _stepsPerRevolution;
    volatile int32_t _encoderCount;
};

