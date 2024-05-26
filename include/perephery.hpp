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

class Position{
public:
    Position(Tachometer &left_tachometer, Tachometer &right_tachometer): _rightTachometer(right_tachometer), _leftTachometer(right_tachometer) {};
    float get_average(void){
        float r = _rightTachometer.get_distance();
        float l = _leftTachometer.get_distance();
        float average = (l + r) / 2.0f;
        return average;
    }
    void set_zero(void){
        _rightTachometer.set_zero();
        _leftTachometer.set_zero();
    }
private:
    Tachometer &_rightTachometer;
    Tachometer &_leftTachometer;
};

class Navigation {

public:
    Navigation() : x(0), y(0), angle(0) {
        memset(_map, false, sizeof(_map));
    }

    // Function to rotate the robot by a given angle
    void rotate(int deltaAngle) {
        angle = (angle + deltaAngle) % 360;
        if (angle < 0) angle += 360;
    }

    void setCurrentCell( void ){
        _map[x][y] = true;
    }
    // Function to move the robot forward by a given distance
    void moveForward(int distance) {
        bool t;
        for (int i = 0; i < distance; ++i) {
            getNextPosition(angle, x, y);
            // Ensure the position is within bounds
            if (x >= 0 && x < 8 && y >= 0 && y < 8) {
                if(i != (distance - 1)){
                    setCurrentCell();
                }
            } else {
                break;
            }
        }
    }

    // Function to get the current position
    void getPosition(int &outX, int &outY, int &outAngle) {
        outX = x;
        outY = y;
        outAngle = angle;
    }

    // Function to check if the front cell is free
    bool isFrontFree() {
        int nextX, nextY;
        getNextPosition(angle, nextX, nextY);
        return (nextX >= 0 && nextX < 8 && nextY >= 0 && nextY < 8 && !(_map[nextX][nextY]));
    }

    // Function to check if the left cell is free
    bool isLeftFree() {
        int nextX, nextY;
        getNextPosition((angle + 270) % 360, nextX, nextY); // Left is 270 degrees counterclockwise
        return (nextX >= 0 && nextX < 8 && nextY >= 0 && nextY < 8 && !(_map[nextX][nextY]));
    }

    // Function to check if the right cell is free
    bool isRightFree() {
        int nextX, nextY;
        getNextPosition((angle + 90) % 360, nextX, nextY); // Right is 90 degrees clockwise
        return (nextX >= 0 && nextX < 8 && nextY >= 0 && nextY < 8 && !(_map[nextX][nextY]));
    }

    bool isCurrentFree() {
        return (_map[x][y]);
    }
    
private:
    void getNextPosition(int currentAngle, int &nextX, int &nextY) {
        nextX = x;
        nextY = y;
        switch (currentAngle) {
            case 0:
                nextX += 1;
                break;
            case 90:
                nextY += 1;
                break;
            case 180:
                nextX -= 1;
                break;
            case 270:
                nextY -= 1;
                break;
        }
    }

    bool _map[8][8]; // 8x8 map
    int x, y;   // Absolute position
    int angle;  // Angle in degrees
};
