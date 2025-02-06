#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

#define speedPinR 9
#define RightDirectPin1 A0
#define RightDirectPin2 A1
#define speedPinL 6
#define LeftDirectPin1 A2
#define LeftDirectPin2 A3
#define LPT 0

#define FAST_SPEED 150
#define SPEED   100 //motor in   speed
#define TURN_SPEED 80
#define BACK_SPEED1 50
#define BACK_SPEED2 80

HUSKYLENS huskylens;
SoftwareSerial mySerial(11, 12);

void printResult(HUSKYLENSResult result);

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);

    pinMode (A0, OUTPUT);
    pinMode (A1, OUTPUT);
    pinMode (A2, OUTPUT);
    pinMode (A3, OUTPUT);

    

int retryCount = 0;
while (!huskylens.begin(mySerial) && retryCount < 10)
{
    Serial.println(F("Begin failed!"));
    delay(100);
    retryCount++;
}
if (retryCount >= 10) {
    Serial.println(F("HUSKYLENS initialization failed. Check wiring."));
}

}

void loop() 
{
    if (!huskylens.request()) 
Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));

    else if(!huskylens.isLearned()) 
Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));

    else if(!huskylens.available()) 
Serial.println(F("No block or arrow appears on the screen!"));

    else
    {
        Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
            driveBot(result);
        }    
    }
}


void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.print("Block: xCenter="); Serial.print(result.xCenter);
        Serial.print(", yCenter="); Serial.print(result.yCenter);
        Serial.print(", width="); Serial.print(result.width);
        Serial.print(", height="); Serial.print(result.height);
        Serial.print(", ID="); Serial.println(result.ID);

    }
    else if (result.command == COMMAND_RETURN_ARROW){
    Serial.print("Arrow:Origin=");Serial.print(result.xOrigin);
    Serial.print("yOrigin=");Serial.print(result.yOrigin);
    Serial.print(",xTarget=");Serial.print(result.xTarget);
    Serial.print(",yTarget=");Serial.print(result.yTarget);
    Serial.print(",ID=");Serial.print(result.ID);
    }

    else{
        Serial.println("Object unknown!");
    }
}
void go_Advance() {  
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(100,100);
   }
void go_Left() { 
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_Motorspeed(100,100);
 }
void go_Right() {   
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(100,100);
   }
void go_Back() {   
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_Motorspeed(100,100);
  }
void stop_Stop() { 
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
 }
 void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L);
  analogWrite(speedPinR,speed_R);  
}



void driveBot(HUSKYLENSResult result)
{
  if(result.xCenter<=120)
  {
    go_Left();
  }

  else if(result.xCenter>=180)
  {
    go_Right();
  }

    else if((result.xCenter>=120)&&(result.xCenter<=180))
  {
    if(result.width<20)
    {
      go_Advance();
    }

    else if(result.width>20)
    {
      stop_Stop();
    }
  }
  
}


