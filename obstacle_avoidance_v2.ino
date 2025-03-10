#define speedPinR 9   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  A0    //  Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2  A1    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  A2    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  A3   ///Left Motor direction pin 1 to MODEL-X IN4
#define LPT 0




/*From left to right, connect to D3,A1-A3 ,D10*/
#define RightObstacleSensor 2  //Right obstacle sensor to D2 (front direction is from arduino point to voltage meter)
#define LeftObstacleSensor 3   //Left obstacle sensor to D3
#define Echo_PIN_1 4 //left sensor
#define Trig_PIN_1 10 //left sensor
#define Echo_PIN_2 5 //right sensor
#define Trig_PIN_2 13 //right sensor


#define FAST_SPEED 250
#define SPEED   120 //motor in   speed
#define TURN_SPEED 200
#define BACK_SPEED1 70
#define BACK_SPEED2 90


int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30;
const int sidedistancelimit = 30;
int distance;
int numcycles=0;
const int turntime =250;
const int backtime = 300;
int thereis;


void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_Motorspeed(BACK_SPEED1,BACK_SPEED1);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L);
  analogWrite(speedPinR,speed_R);  
}

void autostopping()
{
  int IRvalueLeft= digitalRead(RightObstacleSensor);
  int IRvalueRight= digitalRead(LeftObstacleSensor);
 if (IRvalueLeft==LOW && IRvalueRight==LOW)
 { 
  stop_Stop();   //obstacle, stop
     set_Motorspeed(0,0);
     delay(500);
     go_Right();
     
 }
 else  if (IRvalueLeft==HIGH && IRvalueRight==HIGH)
 {
//both sensor detected obstacle, go ahead
      go_Back();
      
    }
 else if (IRvalueLeft==LOW && IRvalueRight==HIGH)
 { 
  stop_Stop();   //obstacle, stop
     set_Motorspeed(0,0);
     delay(500);
     go_Right();
     
 }
  else if (IRvalueLeft==HIGH && IRvalueRight==LOW)
 { 
   stop_Stop();   //obstacle, stop
     set_Motorspeed(0,0);
     delay(500);
     go_Right();
     
 }
 
}


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
//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B100;
    }
 
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
     obstacle_status  =obstacle_status | B1000;
    }
 
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
     obstacle_status  =obstacle_status | B10000;
    }




 
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B100;
    }


 
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | B10;
    }


 
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
    obstacle_status  =obstacle_status | 1;
    }


 
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
 
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}


void auto_avoidance(){


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
    autostopping();
    //set_Motorspeed( SPEED,FAST_SPEED);
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


void setup()
{
 pinMode(speedPinL,OUTPUT); //left motor PWM pin
 pinMode(speedPinR,OUTPUT); //rignt motor PWM  pin
 pinMode(RightDirectPin1,OUTPUT); //left motor direction pin1
 pinMode(RightDirectPin2,OUTPUT); //left motor direction pin2
 pinMode(LeftDirectPin1,OUTPUT); //right motor direction Pin 1
 pinMode(LeftDirectPin2,OUTPUT);  //right motor direction Pin 2
 stop_Stop();
  /*line follow sensors */
 pinMode(RightObstacleSensor,INPUT);
  pinMode(LeftObstacleSensor,INPUT);
 Serial.begin(9600);

  pinMode (A0, OUTPUT);
  pinMode (A1, OUTPUT);
  pinMode (A2, OUTPUT);
  pinMode (A3, OUTPUT);

  pinMode(Trig_PIN_1, OUTPUT); // Set Trig_PIN_1 as OUTPUT
  pinMode(Trig_PIN_2, OUTPUT); // Set Trig_PIN_2 as OUTPUT
  pinMode(Echo_PIN_1, INPUT);  // Set Echo_PIN_1 as INPUT
  pinMode(Echo_PIN_2, INPUT);  // Set Echo_PIN_2 as INPUT


  digitalWrite(Trig_PIN_1, LOW); // Set initial state of Trig_PIN_1 to LOW
  digitalWrite(Trig_PIN_2, LOW); // Set initial state of Trig_PIN_2 to LOW
}

void loop(){
auto_avoidance();
}
