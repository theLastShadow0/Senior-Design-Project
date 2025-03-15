#define speedPinL 6 //Left PWM pin connect MODEL-X ENB
#define speedPinR 9 //RIGHT PWM pin connect MODEL-X ENA

#define LeftDirectPin1  A2  //Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  A3  //Left Motor direction pin 1 to MODEL-X IN4
#define RightDirectPin1 A0  //Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2 A1  //Right Motor direction pin 2 to MODEL-X IN2

#define LPT 2

#define LeftObstacleSensor  3  //Left Infrared Sensor 
#define RightObstacleSensor 2  //Right Infrared Sensor

#define Echo_PIN_1 4           //Left ultrasonic sensor
#define Trig_PIN_1 10          //Left ultrasonic sensor
#define Echo_PIN_2 5           //Right ultrasonic sensor
#define Trig_PIN_2 13          //Right ultrasonic sensor
 
#define FAST_SPEED  250
#define SPEED       75
#define TURN_SPEED  150
#define BACK_SPEED1 70
#define BACK_SPEED2 90


const int distancelimit = 30;
const int sidedistancelimit = 30;
const int turntime = 250;
const int backtime = 300;
int numcycles = 0;
int distance;

void go_Advance(void){
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Left(){
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}
void go_Right(){
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Back(){
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}

void stop_Stop(){
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  set_Motorspeed(0,0);
}

void set_Motorspeed(int speed_L, int speed_R){
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);  
}

void autostopping(){
  int IRvalueLeft = digitalRead(LeftObstacleSensor);
  int IRvalueRight = digitalRead(RightObstacleSensor);

  if (IRvalueLeft == LOW || IRvalueRight == LOW) {
    stop_Stop();
    set_Motorspeed(0,0);
    auto_avoidance(); // Call auto_avoidance when stopped
  } 
  else {
    go_Back();
  }
}

void watch(int &echo_distance_1, int &echo_distance_2){
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
  
  echo_distance_2 = pulseIn(Echo_PIN_2, HIGH); //Measure pulse duration
  echo_distance_2 = echo_distance_2 * 0.01715; //Convert to distance in cm
}

void auto_avoidance(){
  ++numcycles;
  if(numcycles>=LPT){ //Watch if something is around every LPT loops while moving forward
    stop_Stop();
    int echo_distance_1, echo_distance_2;
    watch(echo_distance_1, echo_distance_2);

    Serial.print("begin = ");
    Serial.println(echo_distance_1);
    Serial.println(echo_distance_2);

    if(echo_distance_1 > distancelimit && echo_distance_2 > distancelimit){
      Serial.println("GO FORWARD");
      set_Motorspeed(SPEED,SPEED);
      go_Advance();
      delay(turntime);
      stop_Stop();
    }
    else if(echo_distance_1 < distancelimit){
      Serial.println("SLIGHT TURN RIGHT");
      set_Motorspeed(FAST_SPEED,SPEED);
      go_Advance();
      delay(turntime);
      stop_Stop();
    }

    else if (echo_distance_2 < distancelimit){
      Serial.println("SLIGHT TURN LEFT");
      set_Motorspeed(SPEED,FAST_SPEED);
      go_Advance();
      delay(turntime);
      stop_Stop();
    }
  }
}
void setup(){
  pinMode(speedPinL, OUTPUT);        //left motor PWM pin
  pinMode(speedPinR, OUTPUT);        //right motor PWM  pin
  pinMode(RightDirectPin1, OUTPUT);  //left motor direction pin1
  pinMode(RightDirectPin2, OUTPUT);  //left motor direction pin2
  pinMode(LeftDirectPin1, OUTPUT);   //right motor direction Pin 1
  pinMode(LeftDirectPin2, OUTPUT);   //right motor direction Pin 2
  stop_Stop();

  pinMode(RightObstacleSensor, INPUT);
  pinMode(LeftObstacleSensor, INPUT);
  Serial.begin(9600);

  pinMode (A0, OUTPUT);
  pinMode (A1, OUTPUT);
  pinMode (A2, OUTPUT);
  pinMode (A3, OUTPUT);
 
  pinMode(Trig_PIN_1, OUTPUT);
  pinMode(Trig_PIN_2, OUTPUT);
  pinMode(Echo_PIN_1, INPUT);
  pinMode(Echo_PIN_2, INPUT);
 
  digitalWrite(Trig_PIN_1, LOW); 
  digitalWrite(Trig_PIN_2, LOW); 
}

void loop(){
  auto_avoidance();
}
