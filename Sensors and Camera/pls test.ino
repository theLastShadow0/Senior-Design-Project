#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

#define speedPinL 6  // Left PWM pin connect MODEL-X ENB
#define speedPinR 9  // RIGHT PWM pin connect MODEL-X ENA

#define LeftDirectPin1 A2   // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2 A3   // Left Motor direction pin 1 to MODEL-X IN4
#define RightDirectPin1 A0  // Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2 A1  // Right Motor direction pin 2 to MODEL-X IN2

#define LeftObstacleSensor 3   // Left Infrared Sensor
#define RightObstacleSensor 2  // Right Infrared Sensor

#define Echo_PIN_1 4   // Left ultrasonic sensor
#define Trig_PIN_1 10  // Left ultrasonic sensor
#define Echo_PIN_2 5   // Right ultrasonic sensor
#define Trig_PIN_2 13  // Right ultrasonic sensor

#define FAST_SPEED 200   //150
#define SPEED 150        //100
#define SLOW_SPEED1 90
#define SLOW_SPEED2 80

#define TURN_SPEED1 150  //123
#define TURN_SPEED2 80   //53
#define TURN_SPEED3 170
#define TURN_SPEED4 95  //130
#define SPEED_T1 90
#define SPEED_T2 90
#define SPEED_T3 80
#define SPEED_T4 80

#define BACK_SPEED1 90
#define BACK_SPEED2 80

const int distancelimit = 30;

HUSKYLENS huskylens;
SoftwareSerial mySerial(11, 12);

void printResult(HUSKYLENSResult result);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  pinMode(speedPinL, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);
  stop_Stop();

  pinMode(RightObstacleSensor, INPUT);
  pinMode(LeftObstacleSensor, INPUT);

  pinMode(Trig_PIN_1, OUTPUT);
  pinMode(Trig_PIN_2, OUTPUT);
  pinMode(Echo_PIN_1, INPUT);
  pinMode(Echo_PIN_2, INPUT);

  digitalWrite(Trig_PIN_1, LOW);
  digitalWrite(Trig_PIN_2, LOW);

  int retryCount = 0;
  while (!huskylens.begin(mySerial) && retryCount < 10) {
    Serial.println(F("Begin failed."));
    delay(100);
    retryCount++;
  }

  if (retryCount >= 10) {
    Serial.println(F("HUSKYLENS initialization failed. Check wiring."));
  }
}

void loop() {
  if (!huskylens.request()) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection."));
  }

  else if (!huskylens.isLearned()) {
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one."));
  }

  else if (!huskylens.available()) {
    Serial.println(F("No block or arrow appears on the screen."));
  }

  else {
    Serial.println(F("Object Detected."));
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
      driveBot(result);
    }
  }
}
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.print("Block: xCenter=");
    Serial.print(result.xCenter);
    Serial.print(", yCenter=");
    Serial.print(result.yCenter);
    Serial.print(", width=");
    Serial.print(result.width);
    Serial.print(", height=");
    Serial.print(result.height);
    Serial.print(", ID=");
    Serial.println(result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.print("Arrow:Origin=");
    Serial.print(result.xOrigin);
    Serial.print("yOrigin=");
    Serial.print(result.yOrigin);
    Serial.print(",xTarget=");
    Serial.print(result.xTarget);
    Serial.print(",yTarget=");
    Serial.print(result.yTarget);
    Serial.print(",ID=");
    Serial.print(result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

void left_t() {
  set_Motorspeed(SPEED_T1, SPEED_T2);

  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}

void right_t() {
  set_Motorspeed(SPEED_T3, SPEED_T4);

  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
}

void go_Advance() {
  Serial.println("GO FORWARD");
  set_Motorspeed(SPEED, SPEED);
  
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Slow() {
  Serial.println("SLOW DOWN");
  set_Motorspeed(SLOW_SPEED1, SLOW_SPEED2);
  
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}

void go_Left() {
  Serial.println("TURN LEFT");
  set_Motorspeed(TURN_SPEED3, TURN_SPEED2);
  
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  //digitalWrite(LeftDirectPin1, LOW);
  //digitalWrite(LeftDirectPin2, HIGH);
  left_t();
}

void go_Right() {
  Serial.println("TURN RIGHT");
  set_Motorspeed(TURN_SPEED2, TURN_SPEED1);
  
  //digitalWrite(RightDirectPin1, LOW);
  //digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  right_t();
}

void go_Back() {
  Serial.println("REVERSING");
  set_Motorspeed(BACK_SPEED1, BACK_SPEED2);
  
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}

void stop_Stop() {
  Serial.println("STOP");
  set_Motorspeed(0, 0);
  
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
}

void set_Motorspeed(int speed_L, int speed_R) {
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

void auto_Stopping() {
  int IRvalueLeft = digitalRead(LeftObstacleSensor);
  int IRvalueRight = digitalRead(RightObstacleSensor);

  if (IRvalueLeft == LOW || IRvalueRight == LOW) {
    Serial.println("Infrared Detected Object.");
    stop_Stop();
  } 
  else {
    go_Back();
  }
}

void watch(int &echo_distance_1, int &echo_distance_2) {
  //Sensor 1
  digitalWrite(Trig_PIN_1, LOW);
  delayMicroseconds(5);            //Wait for the trigger to settle
  digitalWrite(Trig_PIN_1, HIGH);  //Send the trigger pulse
  delayMicroseconds(15);           //Pulse length
  digitalWrite(Trig_PIN_1, LOW);   //Stop the pulse

  echo_distance_1 = pulseIn(Echo_PIN_1, HIGH);  //Measure pulse duration
  echo_distance_1 = echo_distance_1 * 0.01715;  //Convert to distance in cm

  //Sensor 2
  digitalWrite(Trig_PIN_2, LOW);
  delayMicroseconds(5);            //Wait for the trigger to settle
  digitalWrite(Trig_PIN_2, HIGH);  //Send the trigger pulse
  delayMicroseconds(15);           //Pulse length
  digitalWrite(Trig_PIN_2, LOW);   //Stop the pulse

  echo_distance_2 = pulseIn(Echo_PIN_2, HIGH);  //Measure pulse duration
  echo_distance_2 = echo_distance_2 * 0.01715;  //Convert to distance in cm
}

void driveBot(HUSKYLENSResult result) {
  int echo_distance_1, echo_distance_2;
  watch(echo_distance_1, echo_distance_2);

  if (result.xCenter <= 120) {
    if (echo_distance_1 > distancelimit) {
      go_Left();
    } else {
      go_Right();
    }
  }
  else if (result.xCenter >= 180) {
    if (echo_distance_2 > distancelimit) {
      go_Right();
    } else {
      go_Left();
    }
  }

  else {
    if (result.width < 45) {  //Tracked Object's block becomes smaller as it moves forward.
      if (echo_distance_1 > distancelimit && echo_distance_2 > distancelimit) {
        go_Advance();
      }
      else if (echo_distance_1 < distancelimit && echo_distance_2 < distancelimit){
        auto_Stopping();
      }
      else if (echo_distance_1 < distancelimit) {
        go_Right();
      } 
      else if (echo_distance_2 < distancelimit) {
        go_Left();
      }
    } 
    else if (result.width <= 60) {  // Slow down instead of stopping 50
      if (echo_distance_1 > distancelimit && echo_distance_2 > distancelimit) {
        go_Slow();
      }
      else if (echo_distance_1 < distancelimit && echo_distance_2 < distancelimit){
        auto_Stopping();
      }
      else if (echo_distance_1 < distancelimit) {
        go_Right();
      } 
      else if (echo_distance_2 < distancelimit) {
        go_Left();
      }
    } 
    else {
      stop_Stop();
      if (result.width > 100) {
        auto_Stopping();
      }
    }
  }
}
