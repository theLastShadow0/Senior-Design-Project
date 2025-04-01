#include <WiFiEspUdp.h>
#include "WiFiEsp.h"
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

WiFiEspUDP Udp;
unsigned int localPort = 8888;  // local port to listen on

#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define speedPinR 9    // RIGHT PWM pin connect MODEL-X ENA

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

#define BUZZER_PIN 8

// WiFi settings
#define SOFT_RX 7    // Softserial RX port for WiFi
#define SOFT_TX 8    // Softserial TX port for WiFi

#define FAST_SPEED 200
#define SPEED 150
#define SLOW_SPEED1 90
#define SLOW_SPEED2 80

#define TURN_SPEED1 133
#define TURN_SPEED2 63
#define TURN_SPEED3 140

#define BACK_SPEED1 90
#define BACK_SPEED2 80

const int distancelimit = 30; // Distance limit for obstacles
const unsigned long AUTO_LEARN_DELAY = 15000;  // 15 seconds delay
bool learningAttempted = false;                // Flag to track if learning has been attempted
unsigned long startTime;

//buzzer
unsigned long previousMillis = 0;
const long beepInterval = 1000;
bool buzzerState = false;

HUSKYLENS huskylens;
SoftwareSerial mySerial(11, 12); 

void printResult(HUSKYLENSResult result);

#define MAX_PACKETSIZE 32    // Serial receive buffer
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;

enum DS
{
  MANUAL_DRIVE,
  AUTO_DRIVE_LF, // HuskyLens tracking (was line follow)
  AUTO_DRIVE_UO  // ultrasonic obstruction
} Drive_Status = MANUAL_DRIVE;

enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  DEF
} Drive_Num = DEF;

String WorkMode = "?";

char ssid[] = "osoyoo_robot";    
char packetBuffer[5];      
int status = WL_IDLE_STATUS;     // the Wifi radio's status

// WiFi software serial
#ifndef HAVE_HWSERIAL1
SoftwareSerial Serial1(SOFT_RX, SOFT_TX); // RX, TX for WiFi
#endif

// use a ring buffer to increase speed and reduce memory allocation
RingBuffer buf(8);

void go_Advance(void)  
{
  Serial.println("GO FORWARD");
  set_Motorspeed(SPEED, SPEED);
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}

void go_Slow() 
{
  Serial.println("SLOW DOWN");
  set_Motorspeed(SLOW_SPEED1, SLOW_SPEED2);
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}

void go_Left() 
{
  Serial.println("TURN LEFT");
  set_Motorspeed(TURN_SPEED3, TURN_SPEED2);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}

void go_Right() 
{
  Serial.println("TURN RIGHT");
  set_Motorspeed(TURN_SPEED2, TURN_SPEED1);
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
}

void go_Back()  
{
  Serial.println("REVERSING");
  set_Motorspeed(BACK_SPEED1, BACK_SPEED2);
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}

void stop_Stop()   
{
  Serial.println("STOP");
  set_Motorspeed(0, 0);
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
}

void set_Motorspeed(int speed_L, int speed_R)
{
  analogWrite(speedPinL, speed_L); 
  analogWrite(speedPinR, speed_R);   
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

// New auto_tracking function that uses HuskyLens
void auto_tracking() {
  if (!huskylens.request()) {
    Serial.println(F("Failed to request data from HUSKYLENS. Check connection."));
  } else if (!huskylens.isLearned()) {
    // Check if it's time to auto-learn and if learning hasn't been attempted yet
    if (!learningAttempted && (millis() - startTime >= AUTO_LEARN_DELAY)) {
      Serial.println(F("Auto-learning now..."));

      // Attempt to learn object as ID 1
      if (huskylens.writeLearn(1)) {
        Serial.println(F("Object learned successfully as ID 1!"));
      } else {
        Serial.println(F("Learning failed. Make sure object is visible to the camera."));
      }

      learningAttempted = true;  // Mark learning as attempted regardless of success
      delay(1000);               // Short delay to allow HuskyLens to process
    }

    // Display countdown if we're waiting to learn
    if (!learningAttempted) {
      unsigned long timeRemaining = (startTime + AUTO_LEARN_DELAY) - millis();
      if (timeRemaining % 1000 < 50) {  // Print roughly once per second
        Serial.print(F("Auto-learning in "));
        Serial.print(timeRemaining / 1000);
        Serial.println(F(" seconds..."));
      }
    }

  } else if (!huskylens.available()) {
    Serial.println(F("No object detected on screen."));
    stop_Stop();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= beepInterval) {
      previousMillis = currentMillis;
      buzzerState = !buzzerState;
      if (buzzerState) {
        tone(BUZZER_PIN, 700);
      } else {
        noTone(BUZZER_PIN);
      }
    }

  } else {
    Serial.println(F("Object detected!"));
    digitalWrite(BUZZER_PIN, LOW);
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
      driveBot(result);
    }
  }

  delay(50);  // Short delay to prevent flooding serial
}

// Can change this function to do something else
void auto_avoidance() {
    stop_Stop();
}

//car motor control
void do_Drive_Tick() {
  if(Drive_Status == MANUAL_DRIVE) {
    switch (Drive_Num) {
      case GO_ADVANCE:
          go_Advance();
          Serial.println("go ahead");
          break;
      case GO_LEFT: 
          go_Left();
          Serial.println("TURN left");
          break;
      case GO_RIGHT:  
          go_Right();
          Serial.println("TURN right");
          break;
      case GO_BACK: 
          go_Back();
          Serial.println("GO back");
          break;
      case STOP_STOP: 
          stop_Stop();
          Serial.println("STOP");
          break;
      default:
          break;
    }
  }
  else if(Drive_Status == AUTO_DRIVE_LF) {
      Serial.println("auto tracking with HuskyLens");
      auto_tracking();
  }
  else if(Drive_Status == AUTO_DRIVE_UO) {
      Serial.println("OBSTACLE AVOID");
      auto_avoidance();
  }
}

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
  } else {
    // Set algorithm to object tracking
    huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);

    // Initialize the timer for auto-learning
    startTime = millis();
    Serial.println(F("System initialized. Auto-learning will begin in 15 seconds."));
    Serial.println(F("Position the object to track in front of the camera."));
  }
  
  // Initialize WiFi
  Serial1.begin(115200);    // initialize serial for ESP module
  Serial1.print("AT+CIOBAUD=9600\r\n");
  Serial1.write("AT+RST\r\n");
  Serial1.begin(9600);    // initialize serial for ESP module
  
  WiFi.init(&Serial1);    // initialize ESP module

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true); // don't continue
  }

  Serial.print("Attempting to start AP ");
  Serial.println(ssid);
 
  status = WiFi.beginAP(ssid, 10, "", 0);

  Serial.println("You're connected to the network");
  Serial.println();

  printWifiStatus();
  
  Udp.begin(localPort);
  
  Serial.print("Listening on port ");
  Serial.println(localPort);
}

void loop() { 
  int packetSize = Udp.parsePacket();
  if (packetSize) {                               // if you get a client,
     Serial.print("Received packet of size ");
    Serial.println(packetSize);
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    char c = packetBuffer[0];
    switch (c)    //serial control instructions
    {   
      case 'A': Drive_Status = MANUAL_DRIVE; Drive_Num = GO_ADVANCE;  WorkMode = "GO_ADVANCE"; break;
      case 'L': Drive_Status = MANUAL_DRIVE; Drive_Num = GO_LEFT; WorkMode = "GO_LEFT"; break;
      case 'R': Drive_Status = MANUAL_DRIVE; Drive_Num = GO_RIGHT; WorkMode = "GO_RIGHT"; break;
      case 'B': Drive_Status = MANUAL_DRIVE; Drive_Num = GO_BACK; WorkMode = "GO_BACK"; break;
      case 'E': Drive_Status = MANUAL_DRIVE; Drive_Num = STOP_STOP; WorkMode = "STOP_STOP"; break;
      case 'O': Drive_Status = AUTO_DRIVE_UO; Serial.println("go OBSTACLE"); WorkMode = "OBSTACLE"; break;
      case 'T': Drive_Status = AUTO_DRIVE_LF; WorkMode = "HuskyLens tracking"; break;
      default: break;
    } //END OF ACTION SWITCH
  }
  do_Drive_Tick();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
