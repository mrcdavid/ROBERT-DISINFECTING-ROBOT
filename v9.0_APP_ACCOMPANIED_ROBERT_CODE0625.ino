#include <EEPROM.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <Servo.h>

AF_DCMotor motor1 (1, MOTOR12_1KHZ);  //create motor #1 using M1 output on Motor Driver Shield, set to 1Khz PWM frequency
AF_DCMotor motor2 (2, MOTOR12_1KHZ);  //create motor #2 using M1 output on Motor Driver Shield, set to 1Khz PWM frequency
AF_DCMotor motor3 (3, MOTOR34_1KHZ);  //create motor #3 using M1 output on Motor Driver Shield, set to 1Khz PWM frequency
AF_DCMotor motor4 (4, MOTOR34_1KHZ);  //create motor #4 using M1 output on Motor Driver Shield, set to 1Khz PWM frequency

#define TRIG_PIN A0         // set the pin for trigpin in A0
#define ECHO_PIN A1         // set the pin for echopin IN A1
#define MAX_DISTANCE 120    // set the distance of detection for ultrasonic 

NewPing sonar(TRIG_PIN,ECHO_PIN,MAX_DISTANCE);    // for ultrasonic sensors 
SoftwareSerial BT(0,1);                           // for bluetooth module pins

Servo myServo;
bool updateMemory =false;
int distanceF = 100;
int distanceR = 0;
int distanceL = 0;
int i = 160;                        //sets Motor speed
int j = 300;                        //sets turn angle
int k = 0;
int range;
int address = 0;                   // for trace path and play path loop
int address_Start = 1;             // initial value for trace path and play path loop
int address_End = 0;               // end value for trace path and play path loop
char cmd;                       // for bluetooth
int relayForUVC = 9;           // set the pin 9 for UVC relay
unsigned long timeStart = 0;    // for millis() function
long interval = 30000;          // 30000 = 30 seconds, time sets for rover


void setup()                           // put your setup code here, to run once:
{
  BT.begin(9600);
  myServo.attach(10);                 // the Motor Shield Servo1 pin corresponds to D2 of arduino Uno Board
  myServo.write(90);                  // Servo motor position itself to 90 degrees

  pinMode(relayForUVC, OUTPUT);
  digitalWrite(relayForUVC, HIGH);
  
  distanceF = readPing();
  delay(2000);
  range = MAX_DISTANCE-20;
  delay(100);
  k = EEPROM.read(200);
  
}

void Stop()
{ 
    motor1.run(RELEASE); 
    motor4.run(RELEASE); 
    motor2.run(RELEASE); 
    motor3.run(RELEASE); 
}

int readPing()
{
    int cm = sonar.ping_cm();
    delay(200);
    if(cm == 0)
    {
      cm = range;
    }
    return cm;
}

void goBackward()
{ 
    motor1.run(BACKWARD); 
    motor4.run(BACKWARD); 
    motor2.run(BACKWARD); 
    motor3.run(BACKWARD); 

    motor1.setSpeed(i-26);
    motor4.setSpeed(i-26);
    motor2.setSpeed(i-26);
    motor3.setSpeed(i-26);
    delay(400);
    Stop();
}


void goForward()
{
    motor1.run(FORWARD); 
    motor4.run(FORWARD); 
    motor2.run(FORWARD); 
    motor3.run(FORWARD); 

    motor1.setSpeed(i-26);
    motor4.setSpeed(i-26);
    motor2.setSpeed(i-26);
    motor3.setSpeed(i-26);
    delay(1000);
    Stop();
}

void goSmart()
{
  timeStart = millis();
  j = 400;
  int b = 150;
  int angle = 500;
  while((millis() - timeStart) < interval)
  {
    motor1.run(FORWARD); 
    motor4.run(FORWARD); 
    motor2.run(FORWARD); 
    motor3.run(FORWARD); 

    motor1.setSpeed(i-26);
    motor4.setSpeed(i-26);
    motor2.setSpeed(i-26);
    motor3.setSpeed(i-26);
    delay(100);
    distanceF = readPing();

    if(distanceF < range-45)
    {
      // Stop();
      motor1.run(RELEASE); 
      motor4.run(RELEASE); 
      motor2.run(RELEASE); 
      motor3.run(RELEASE); 
      delay(400);
      
      // goBackward();
      motor1.run(BACKWARD); 
      motor4.run(BACKWARD); 
      motor2.run(BACKWARD); 
      motor3.run(BACKWARD); 
      motor1.setSpeed(i-26);
      motor4.setSpeed(i-26);
      motor2.setSpeed(i-26);
      motor3.setSpeed(i-26);
      delay(400);
      
      // Stop();
      motor1.run(RELEASE); 
      motor4.run(RELEASE); 
      motor2.run(RELEASE); 
      motor3.run(RELEASE); 
      delay(400);
      
      // takeTurn();
      distanceR = lookRight();
      delay(200);
      distanceL = lookLeft();
      delay(200);
        if(distanceR >= distanceL) 
          {
            while(distanceF < range)
            {
              // turnRight();
              motor4.run(BACKWARD); 
              motor4.setSpeed(b);
              motor1.run(FORWARD); 
              motor1.setSpeed(b);
              motor3.run(BACKWARD); 
              motor3.setSpeed(b);
              motor2.run(FORWARD); 
              motor2.setSpeed(b);
              delay(angle);
              
              // Stop();
              motor1.run(RELEASE); 
              motor4.run(RELEASE); 
              motor2.run(RELEASE); 
              motor3.run(RELEASE); 
              delay(400);
              
              distanceF = readPing();
              if(distanceF = range)
              {
                continue;
              }
            }
          }else if(distanceL >= distanceR)
          {
            while(distanceF < range)
            {
              // turnLeft();
              motor1.run(BACKWARD); 
              motor1.setSpeed(b);
              motor4.run(FORWARD); 
              motor4.setSpeed(b);
              motor2.run(BACKWARD); 
              motor2.setSpeed(b);
              motor3.run(FORWARD); 
              motor3.setSpeed(b);
              delay(angle);
              
              // Stop();
              motor1.run(RELEASE); 
              motor4.run(RELEASE); 
              motor2.run(RELEASE); 
              motor3.run(RELEASE); 
              delay(400);
              distanceF = readPing();
              if(distanceF = range)
              {
                continue;
              }
            }
          }      
        }
    }
   Stop();
}

void turnRight()
{ 
    int b = 150;
    int angle = 300;
    motor4.run(BACKWARD); 
    motor4.setSpeed(b);
    motor1.run(FORWARD); 
    motor1.setSpeed(b);
    motor3.run(BACKWARD); 
    motor3.setSpeed(b);
    motor2.run(FORWARD); 
    motor2.setSpeed(b);
    delay(angle);
    Stop();
}

void turnLeft()
{ 
    int b = 150;
    int angle = 300;
    motor1.run(BACKWARD); 
    motor1.setSpeed(b);
    motor4.run(FORWARD); 
    motor4.setSpeed(b);
    motor2.run(BACKWARD); 
    motor2.setSpeed(b);
    motor3.run(FORWARD); 
    motor3.setSpeed(b);
    delay(angle);
    Stop();
}

void takeTurn()
{
  j = 400;
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);
  if(distanceR >= distanceL) 
  {
    while(distanceF < range)
    {
      turnRight();
      distanceF = readPing();
      if(distanceF = range)
      {
        goSmart();
        }
    }
  }
  else if(distanceL >= distanceR)
  {
    while(distanceF < range)
    {
      turnLeft();
      distanceF = readPing();
      if(distanceF = range)
      {
        goSmart();
        }
      }
    }              
}
              
int lookRight()
{
  myServo.write(25);
  int distanceR = readPing();
  delay(700);
  myServo.write(90);
  return distanceR;  
}

int lookLeft()
{
  myServo.write(155);
  int distanceL = readPing();
  delay(700);
  myServo.write(90);
  return distanceL;  
}

void memory_IN()
{
  EEPROM.update(address,cmd);
  EEPROM.write(address_Start,address);
  address = address + 1;
}

void goPath()
{
  for(address=address_Start; address <= address_End; address += 1)
  {
    cmd = EEPROM.read(address);
    if(cmd == 'F')
    {
      goForward();
      delay(1000);
    }
    else if(cmd == 'B')
    {
      goBackward();
      delay(400); 
    }
    else if(cmd == 'R')
    {
      turnRight();
      delay(340);
    }
    else if(cmd == 'L')
    {
      turnLeft();
      delay(340);
    }
  }
}

void memory_OUT()
{
  goPath();
  delay(3000);
}

void UVC_ON()                // ON
{
  digitalWrite(relayForUVC,LOW);
}
void UVC_OFF()                   // OFF
{
  digitalWrite(relayForUVC,HIGH); 
}
    
void loop()                          // put your main code here, to run repeatedly:
{
  while(BT.available())  
  {
    cmd = BT.read();
    if(cmd == 'F')          //move in Forward direction    
    {
      BT.println("Go Forward");
      goForward();
      if(updateMemory)
      {
        memory_IN();
      } 
    }    
    
    else if(cmd == 'B')          //move in Backward direction    
    {
      BT.println("Go Backward");
      goBackward(); 
      if(updateMemory)
      {
        memory_IN();
      }
    }   
    
    else if(cmd == 'R')         //take 90 degree Right turn
    {
      BT.println("Turn Right");
      j = 340;
      turnRight(); 
      if(updateMemory)
      {
        memory_IN();
      }
    } 
    
    else if(cmd == 'L')         //take 90 degree Left turn
    {
      BT.println("Turn Left");
      j = 340;
      turnLeft(); 
      if(updateMemory)
      {
        memory_IN();
      }
    } 
    
    else if(cmd == 'Q')                 // ON
    {
     UVC_ON();
    }
    
     else if(cmd == 'K')                // OFF
    {
     UVC_OFF();     
    }

    else if(cmd == 'T')               //to Start memory IN to Trace Path1
    {
      BT.println("Record the Path");
      updateMemory = true;
      address = 1;
      address_Start = 0; 
      i = 180;
    }
    
    else if(cmd == 'G')          //to Start memory OUT to Retrace Path1 i.e., Room1
    {
      BT.println("Go Play Path");
      updateMemory = false;
      i = 175;
      address_End=EEPROM.read(0);
      address_Start = 1;
      memory_OUT();
    }
    
    else if(cmd == 'A')           //move in Rover action i.e., goSmart
    {
      BT.println("Go Smart");
      j = 100;       
      goSmart();   
    }
  }
}  
