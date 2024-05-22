#include <Arduino.h>
#include "perephery.hpp"


// r - 5 13
// Pin definitions (adjust as needed)
const int motorLeftSPin = 5;
const int motorLeftAPin = 4;
const int motorLeftBPin = 12;
const int motorRightSPin = 13;
const int motorRightAPin = 8;
const int motorRightBPin = 10;
const int irSensorPins[5] = {6, 11, 7, 3, 2};
const int encoderLeftPin = 0;
const int encoderRightPin = 1;

// Params
const int16_t speed_line_folowing = 100;

Tachometer left_encoder(encoderLeftPin, 198.0f, 40);
Tachometer right_encoder(encoderRightPin, 198.0f, 40);
TB6612Driver motorDriver(motorRightSPin, motorRightAPin, motorRightBPin, motorLeftSPin, motorLeftAPin, motorLeftBPin);
IRSensorArray line_block(irSensorPins[0], irSensorPins[1], irSensorPins[2], irSensorPins[3], irSensorPins[4]);

bool follow_line(void) {
  static uint16_t tttt = 0;
  bool stop_ok = true;
  while (1){
    line_sensors_t line = line_block.get_line();

    if (line.RR || line.LL) {
      break;
    }

    if (!line.LC && line.CC && !line.RC) {
        motorDriver.set_power(speed_line_folowing, speed_line_folowing); // Move forward
    } else if (line.LC && line.CC && !line.RC) {
        motorDriver.set_power(speed_line_folowing/2, speed_line_folowing); // Turn right
    } else if (!line.LC && line.CC && line.RC) {
        motorDriver.set_power(speed_line_folowing, speed_line_folowing/2); // Turn left
    } else if (line.LC && !line.CC && !line.RC) {
        motorDriver.set_power(0, speed_line_folowing);
    } else if (!line.LC && !line.CC && line.RC) {
        motorDriver.set_power(speed_line_folowing, 0);
    }
    if (!line.LC && !line.CC && !line.RC){
      motorDriver.set_power(speed_line_folowing/2, speed_line_folowing/2);
      tttt++;
    }else{
      tttt = 0;
    }

    if(tttt > 100){
      tttt = 0;
      break;
    }
    delay(1);
  }
  
  delay(1);
  motorDriver.set_power(0, 0);
  return stop_ok;
}

void left_encoder_isr(void){
  left_encoder.increment_ISR();
}

void right_encoder_isr(void){
  right_encoder.increment_ISR();
}

typedef enum{
    CROSS_NONE = 0b000,
    CROSS_F    = 0b010,
    CROSS_L    = 0b100,
    CROSS_R    = 0b001,
    CROSS_LR   = 0b101,
    CROSS_LF   = 0b110,
    CROSS_RF   = 0b011,
} crossroad_flags_t;

crossroad_flags_t detect_crossroad( void ){
  uint8_t cross = CROSS_NONE;
  
  if(line_block.get_line().all == 0x00){
    return CROSS_NONE;
  }
  if(line_block.get_line().LL){
    cross |= CROSS_L;
  }
  if(line_block.get_line().RR){
    cross |= CROSS_R;
  }
  motorDriver.set_power(speed_line_folowing, speed_line_folowing);
  while (line_block.get_line().LL || line_block.get_line().RR){
    delay(1);
  }
  delay(10);
  if(line_block.get_line().CC){
    cross |= CROSS_F;
  }
  motorDriver.set_power(0, 0);
  return (crossroad_flags_t)cross;
}

void setup() {
  Serial.begin(115200);
  left_encoder.init(left_encoder_isr);
  right_encoder.init(right_encoder_isr);
  motorDriver.init();
  line_block.init();

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //motorDriver.set_power(50, 50);
}

void turn_left( void ){
  motorDriver.set_power(speed_line_folowing, speed_line_folowing);
  delay(100);
  motorDriver.set_power(-speed_line_folowing, speed_line_folowing);
  delay(100);
  while (!line_block.get_line().CC);
  motorDriver.set_power(0, 0);
}

void turn_right( void ){
  motorDriver.set_power(speed_line_folowing, speed_line_folowing);
  delay(100);
  motorDriver.set_power(speed_line_folowing, -speed_line_folowing);
  delay(100);
  while (!line_block.get_line().CC);
  motorDriver.set_power(0, 0);
}

void turn_back( void ){
  motorDriver.set_power(speed_line_folowing, speed_line_folowing);
  delay(100);
  motorDriver.set_power(speed_line_folowing, -speed_line_folowing);
  delay(400);
  while (!line_block.get_line().CC);
  motorDriver.set_power(0, 0);
}











void loop() {
  /*
  while (1){
    float a = left_encoder.get_distance();
    float b = right_encoder.get_distance();

    Serial.print(a);
    Serial.print("\t");
    Serial.println(b);
    delay(100);
  }*/
  

  follow_line();
  

  crossroad_flags_t cross = detect_crossroad();
  Serial.print("CROSS TYPE:");
  Serial.println(cross);

  if(cross == CROSS_L){
    turn_left();
  }
  if(cross == CROSS_R){
    turn_right();
  }
  if(cross == CROSS_NONE){
    turn_back();
  }


  delay(1000);
}
