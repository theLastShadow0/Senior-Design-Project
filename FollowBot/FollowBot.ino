#include "HUSKYLENS.h"
#include <SoftwareSerial.h>

// Define motor pins (as before)
#define speedPinR 9
#define RightDirectPin1 12
#define RightDirectPin2 11
#define speedPinL 6
#define LeftDirectPin1 7
#define LeftDirectPin2 8
#define LPT 0
#define SOFT_RX 0
#define SOFT_TX 1

// Define obstacle sensor pins (as before)
#define RightObstacleSensor 2
#define LeftObstacleSensor 3
#define Echo_PIN_1 4
#define Trig_PIN_1 10
#define Echo_PIN_2 5
#define Trig_PIN_2 13

#define FAST_SPEED 150
#define SPEED   100
#define TURN_SPEED 80
#define BACK_SPEED1 100
#define BACK_SPEED2 80

// HuskyLens setup
SoftwareSerial mySerial(SOFT_RX, SOFT_TX); // RX, TX
HUSKYLENS huskylens;

// Variables
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 200;
const int sidedistancelimit = 100;
int distance;
int numcycles = 0;
const int turntime = 250;
const int backtime = 300;
int thereis;

// Motor control functions remain unchanged
void go_Advance() {  
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW); }
void go_Left() { 
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
 }
void go_Right() {   
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW); }
void go_Back() {   
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);}
void stop_Stop() { 
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
 }
void set_Motorspeed(int speed_L, int speed_R) { /* ... */ }

// Watch and obstacle avoidance functions remain unchanged
int watch() { 
  long echo_distance_1;
  long echo_distance_2;

  // Sensor 1 (Front sensor)
  digitalWrite(Trig_PIN_1, LOW);
  delayMicroseconds(5);  // Wait for the trigger to settle
  digitalWrite(Trig_PIN_1, HIGH);  // Send the trigger pulse
  delayMicroseconds(15);  // Pulse length
  digitalWrite(Trig_PIN_1, LOW);  // Stop the pulse

  echo_distance_1 = pulseIn(Echo_PIN_1, HIGH);  // Measure pulse duration
  echo_distance_1 = echo_distance_1 * 0.01657;  // Convert to distance in cm

  // Sensor 2 (Side or other sensor)
  digitalWrite(Trig_PIN_2, LOW);
  delayMicroseconds(5);  // Wait for the trigger to settle
  digitalWrite(Trig_PIN_2, HIGH);  // Send the trigger pulse
  delayMicroseconds(15);  // Pulse length
  digitalWrite(Trig_PIN_2, LOW);  // Stop the pulse

  echo_distance_2 = pulseIn(Echo_PIN_2, HIGH);  // Measure pulse duration
  echo_distance_2 = echo_distance_2 * 0.01657;  // Convert to distance in cm

  // Optionally, you can return either distance or both distances.
  // For example, to return the minimum distance (to the closest obstacle):
  int min_distance = min(echo_distance_1, echo_distance_2);

  // Return the smallest distance, or you can modify this to return both distances
  return min_distance;

 }
String watchsurrounding() { 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B100;
    }
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
     obstacle_status  =obstacle_status | B1000;
    }
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
     obstacle_status  =obstacle_status | B10000;
    }


  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B100;
    }

  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B10;
    }

  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | 1;
    }

  delay(300);
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status

 }
void auto_avoidance() { 
  ++numcycles;
  if(numcycles>=LPT){ //Watch if something is around every LPT loops while moving forward 
     stop_Stop();
    String obstacle_sign=watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
      Serial.print("begin str=");
        Serial.println(obstacle_sign);
                    if( obstacle_sign=="10000"){
     Serial.println("SLIT right");
          set_Motorspeed(FAST_SPEED,SPEED);
     go_Advance();
 
      delay(turntime);
      stop_Stop();
    }
        else    if( obstacle_sign=="00001"  ){
     Serial.println("SLIT LEFT");
       set_Motorspeed(SPEED,FAST_SPEED);
      go_Advance();
  
      delay(turntime);
      stop_Stop();
    }
    else if( obstacle_sign=="11100" || obstacle_sign=="01000" || obstacle_sign=="11000"  || obstacle_sign=="10100"  || obstacle_sign=="01100" ||obstacle_sign=="00100"  ||obstacle_sign=="01000" ){
     Serial.println("hand right");
	    go_Right();
      set_Motorspeed(TURN_SPEED,TURN_SPEED);
      delay(turntime);
      stop_Stop();
    } 
    else if( obstacle_sign=="00010" || obstacle_sign=="00111" || obstacle_sign=="00011"  || obstacle_sign=="00101" || obstacle_sign=="00110" || obstacle_sign=="01010" ){
    Serial.println("hand left");
     go_Left();//Turn left
     set_Motorspeed(TURN_SPEED,TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }
 
    else if(  obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  ){
    Serial.println("hand back right");
	  go_Left();
		set_Motorspeed( FAST_SPEED,SPEED);
       delay(backtime);
          stop_Stop();
        } 
         else if( obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
    Serial.println("hand back left");
    go_Right();
    set_Motorspeed( SPEED,FAST_SPEED);
       delay(backtime);
          stop_Stop();
        }    
  
        else Serial.println("no handle");
    numcycles=0; //Restart count of cycles
  } else {
     set_Motorspeed(SPEED,SPEED);
     go_Advance();  // if nothing is wrong go forward using go() function above.
        delay(backtime);
          stop_Stop();
  }
  
  //else  Serial.println(numcycles);
  
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
 Serial.println("final go back");
    go_Right();
    set_Motorspeed( SPEED,FAST_SPEED);
  delay(backtime*3/2);
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
  Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis=0;
  }
 }

// HuskyLens object-following function
void follow_object() {
  if (!huskylens.request()) {
    Serial.println("Failed to request data from HuskyLens!");
    return;
  }

  if (!huskylens.isLearned()) {
    Serial.println("No object learned yet!");
    auto_avoidance(); // Switch to obstacle avoidance
    return;
  }

  if (!huskylens.available()) {
    Serial.println("No object detected!");
    auto_avoidance(); // Switch to obstacle avoidance
    return;
  }

  // Get the first tracked object
  HUSKYLENSResult result = huskylens.read();
  int x_center = result.xCenter; // X-coordinate of the object's center

  // Control car movement based on object's position
  if (x_center < 140) { // Object is on the left
    go_Left();
    set_Motorspeed(TURN_SPEED, TURN_SPEED);
  } else if (x_center > 180) { // Object is on the right
    go_Right();
    set_Motorspeed(TURN_SPEED, TURN_SPEED);
  } else { // Object is centered
    go_Advance();
    set_Motorspeed(SPEED, SPEED);
  }
}

void auto_stopping(){
 int IRvalueLeft= digitalRead(LeftObstacleSensor);
  int IRvalueRight=digitalRead(RightObstacleSensor);
 if (IRvalueLeft==LOW || IRvalueRight==LOW)
 { 
  stop_Stop();
    set_Motorspeed(0,0);
 }
 else  if (IRvalueLeft==HIGH || IRvalueRight==HIGH)
 {
  go_Back();
    }
 else if (IRvalueLeft==LOW || IRvalueRight==HIGH)
 { 
  stop_Stop();
    set_Motorspeed(0,0);
 }
  else if (IRvalueLeft==HIGH || IRvalueRight==LOW)
 { 
    stop_Stop();
      set_Motorspeed(0,0);
 }
}

void setup() {
  // Initialize motor and sensor pins
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

  // Initialize serial communication
  Serial.begin(9600);
  mySerial.begin(9600); // HuskyLens communication

  // Initialize HuskyLens
  if (!huskylens.begin(mySerial)) {
    Serial.println("HuskyLens initialization failed!");
    while (1);
  }
  Serial.println("HuskyLens initialized!");

  // Set HuskyLens to object-tracking mode
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
}

void loop() {
  follow_object(); // Prioritize object following
  auto_stopping(); // Check for immediate obstacles
}
