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
const float sensorLineOffset_mm = 50.0f;

// Params
const int16_t speed_line_folowing = 100;

Tachometer left_encoder(encoderLeftPin, 198.0f, 40);
Tachometer right_encoder(encoderRightPin, 198.0f, 40);
TB6612Driver motorDriver(motorRightSPin, motorRightAPin, motorRightBPin, motorLeftSPin, motorLeftAPin, motorLeftBPin);
IRSensorArray line_block(irSensorPins[0], irSensorPins[1], irSensorPins[2], irSensorPins[3], irSensorPins[4]);
Position position(left_encoder, right_encoder);
Navigation navigation;

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
      motorDriver.set_power(speed_line_folowing, speed_line_folowing);
      tttt++;
    }else{
      tttt = 0;
    }

    if(tttt > 200){
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
    CROSS_NONE = 0b0000,
    CROSS_F    = 0b0010,
    CROSS_L    = 0b0100,
    CROSS_R    = 0b0001,
    CROSS_LR   = 0b0101,
    CROSS_LF   = 0b0110,
    CROSS_RF   = 0b0011,
    CROSS_LRF  = 0b0111,
    CROSS_END  = 0b1000,
} crossroad_flags_t;

crossroad_flags_t detect_crossroad( void ){
  uint8_t cross = CROSS_NONE;
  float dist_start = position.get_average();
  if(line_block.get_line().all == 0x00){
    return CROSS_NONE;
  }
  
  motorDriver.set_power(speed_line_folowing, speed_line_folowing);
  while (line_block.get_line().LL || line_block.get_line().RR){
    delay(1);
    if(line_block.get_line().LL){
      cross |= CROSS_L;
    }
    if(line_block.get_line().RR){
      cross |= CROSS_R;
    }
  }
  delay(25);
  if(line_block.get_line().CC || line_block.get_line().LC || line_block.get_line().RC){
    cross |= CROSS_F;
  }
  motorDriver.set_power(0, 0);
  float dist_end = position.get_average();
  if((dist_end - dist_start) > 50.0f){
    cross = CROSS_END;
  }
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
delay(5000);
  //motorDriver.set_power(50, 50);
}

const int spd = (int)((float)speed_line_folowing*0.7);

void turn_left( void ){
  motorDriver.set_power(spd, spd);
  delay(250);
  motorDriver.set_power(0, 0);
  delay(200);
  motorDriver.set_power(-spd, spd);
  delay(200);
  while (!line_block.get_line().CC){
    delay(1);
  }
  motorDriver.set_power(0, 0);
  delay(200);
  navigation.rotate(-90);
}

void turn_right( void ){
  motorDriver.set_power(spd, spd);
  delay(250);
  motorDriver.set_power(0, 0);
  delay(200);
  motorDriver.set_power(spd, -spd);
  delay(200);
  while (!line_block.get_line().CC){
    delay(1);
  }
  motorDriver.set_power(0, 0);
  delay(200);
  navigation.rotate(90);
}

void turn_back( void ){
  motorDriver.set_power(spd, spd);
  delay(250);
  motorDriver.set_power(0, 0);
  delay(200);
  motorDriver.set_power(spd, -spd);
  delay(800);
  while (!line_block.get_line().CC){
    delay(1);
  }
  motorDriver.set_power(0, 0);
  delay(200);
  navigation.rotate(180);
}


crossroad_flags_t move_to_next_crossroad( void ){
  while (1){
    position.set_zero();
    do{
      follow_line();
    }while (position.get_average() <= 100.0f);

    crossroad_flags_t cross = detect_crossroad();
    if(cross == CROSS_END){
        while (1);
        
        return cross;
    }
    //Serial.print("CROSS TYPE:");
    //Serial.println(cross);
    
    float dist = position.get_average();
    Serial.println(dist);
    navigation.moveForward(roundf(dist/200.0f));

    
    int x, y, a;
    navigation.getPosition(x, y, a);
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.println(a);
    
    delay(500);
    if(cross == CROSS_L){
      Serial.println("turn left");
      turn_left();
    }else if(cross == CROSS_R){
      Serial.println("turn right");
      turn_right();
    }else{
      return cross;
    }
    
    delay(500);
  }
}


bool find_finish( void ){
  crossroad_flags_t crossboard_type;
  crossboard_type = move_to_next_crossroad();
	
  if(crossboard_type == CROSS_END){
    return true;
  }

	if(navigation.isCurrentFree() || (crossboard_type == CROSS_NONE)){
		Serial.println("TURN BACK");		
		turn_back();
		move_to_next_crossroad();
		Serial.println("TURN BACK FINISH");
		return false;
	}else{
		navigation.setCurrentCell();
	}
	
	bool find = false;
	bool reversed = false;
	
	
	bool right_available = navigation.isRightFree();
	bool left_available = navigation.isLeftFree();
	bool front_available = navigation.isFrontFree();

    if(crossboard_type == CROSS_LF){
		Serial.println("LF");
		if(front_available){
			Serial.println("Go to front");
			find = find_finish();
			reversed = true;
		}
		if(left_available && !find){
			Serial.println("Go to left");
			if(reversed){
				turn_right();
			}else{
				turn_left();
			}
			find = find_finish();
			turn_right();
		}
		if(!front_available && !left_available && !find){
			turn_back();
		}
    }else if(crossboard_type == CROSS_RF){
		Serial.println("RF");
		if(front_available){
			Serial.println("Go to front");
			find = find_finish();
			reversed = true;
		}
		if(right_available && !find){
			Serial.println("Go to right");
			if(reversed){
				turn_left();
			}else{
				turn_right();
			}
			find = find_finish();
			turn_left();
		}
		if(!front_available && !right_available && !find){
			turn_back();
		}
    }else if(crossboard_type == CROSS_LRF){
		bool direct = false;
		Serial.println("RLF");
        if(front_available){
			Serial.println("Go to front");
			find = find_finish();
			reversed = true;
		}
		if(right_available && !find){
			Serial.println("Go to right");
			if(reversed){
				turn_left();
			}else{
				turn_right();
			}
			find = find_finish();
			if(find){
				turn_left();
			}
			direct = true;
		}
		if(left_available && !find){
			if(direct){
				// Nope
			}else if(reversed){
				turn_right();
			}else{
				turn_left();
			}
			Serial.println("Go to left");
			find = find_finish();
			turn_right();
		}else if(!left_available && right_available && !find){
			turn_left();
		}
		if(!front_available && !right_available && !left_available){
			turn_back();
		}
    }else if(crossboard_type == CROSS_LR){
		bool direct = false;
		Serial.println("RL");
		if(right_available){
			Serial.println("Go to right");
			turn_right();
			find = find_finish();
			if(find){
				turn_left();
			}
			direct = true;
		}
		if(left_available && !find){
			if(!direct){
				turn_left();
			}
			Serial.println("Go to left");
			find = find_finish();
			turn_right();
		}else if(!left_available && !find){
			turn_left();
		}
		if(!right_available && !left_available && !find){
			turn_back();
		}
	}
	
	if(!find){
		Serial.println("NOT FOUND");
	}else{
		Serial.println("PATH FOUND!");
	}
  
	return find;
}

void loop() {
  find_finish();
  delay(10000000);
}
