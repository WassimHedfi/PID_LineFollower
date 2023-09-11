/* This code is typed line by line, letter by letter
By Wassim HEDFI in February 2022
It is not under any copyright and shared to be used, edited and shared even more

The Code is for a Linefollower robot.
The robot is a 2wd Robot using 7 IR sensors and PID Regulation

To make it work on your robot, you will have to change and fineTune the constants in the "Constants for PID and Motors" Part.
        You start by fixing the 'motor1speed', 'motor2speed' and 'OutputFactor'
        Then, keep 'ki' and 'kd' null and start with a small value (ex:0.01) for 'kp' until the robot starts following the line
        After that you will start increasing 'ki', starting from a small value (ex:0.0001) until you see the smooth PID effect 
        'kd' is optional and might be of unpleasant effect depending on the robots design and the road to be navigated. In case you will use it , start from 0.001
        ==NB== Remember the constants can very big time depending on endless factors, you should try and see dor urself, a documentation on PID Regulation might be very helpful.

The code is also setup to be used on a non PID regulation robot (if statements)
Enjoy and explore..
Never ceize the Adventure..
   HEDFI Wassim.
*/


#include <Wire.h>


//____________________________________________________________________________INSTANCES
int Black =  0;
int White =  1;




int sensorr =      9    ;   // for non PID circulation
int sensorl =        2   ;    // for non PID circulation
int sensorL ;
int sensorR;


#define MAX_MOTOR_SPEED       255
#define NORMAL_MOTOR_SPEED    80
#define SLOW_MOTOR_SPEED      80
#define turn_speed           120
#define forward_speed        100



// Pins for IR sensors
#define sensor1          40
#define sensor2          42
#define sensor3          44
#define sensor4          46
#define sensor5          52
#define sensor6          50
#define sensor7          48

int irReadings[7];

// Pins for Motors ___________________________________________
int motor1Forward =   7;                                    //|
int motor1Backward = 6;                                     //|                             
int motor1pwmPin = 5;                                       //|
int motor2Forward = 8;                                      //|
int motor2Backward = 9;                                     //|
int motor2pwmPin = 10;                                      //|
                                                            //|
// For No PID circuation                                    //|
int ENABLE_1_PIN =  5;                                      //|
int MOTOR_1_INPUT_1  = 6  ;                                 //|                                    
int MOTOR_1_INPUT_2 = 7 ;                                   //|
int MOTOR_2_INPUT_1 = 8 ;                                   //|
int MOTOR_2_INPUT_2  = 9 ;                                  //|
int ENABLE_2_PIN     = 10 ;                                 //|
//____________________________________________________________|



// Constants for PID and Motors_______________________________
float pTerm, iTerm, dTerm;                                  //|
int error;                                                  //|
int previousError;                                          //|
float  kp = 0.12; //                                         //|
float ki = 0.00002;                                          //|
float kd = 0.002; //                                         //|
float output;                                               //|
int integral,  derivative;                                  //|  
                                                            //|
int motor1newSpeed;                                         //|
int motor2newSpeed;                                         //|
int motor2Speed = 80;                                      //|
int motor1Speed = 80;                                      //|
                                                            //|
int OutputFactor = 40;                                       //|
//____________________________________________________________|

//_____________________________________________________________________________________________________________________
//________________________________________________           __________________________________________________________
//________________________________________________   SETUP   __________________________________________________________
//________________________________________________           __________________________________________________________
//_____________________________________________________________________________________________________________________
void setup() {
  Wire.begin();
  Wire.setClock(300000);

  //Declare all IR sensors  as inputs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  pinMode(sensor7, INPUT);

  //Declare Motors pins as outputs
  pinMode(motor1Forward, OUTPUT);
  pinMode(motor1Backward,  OUTPUT);
  pinMode(motor1pwmPin, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor2Backward, OUTPUT);
  pinMode(motor2pwmPin, OUTPUT);

}
//______________________________________________________________________________________________________________________
//________________________________________________            __________________________________________________________
//________________________________________________   CHUNKS   __________________________________________________________
//________________________________________________            __________________________________________________________
//______________________________________________________________________________________________________________________




//____________________________________________              _______________________________________________
//____________________________________________  IR Sensors  _______________________________________________
//____________________________________________              _______________________________________________
void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  irReadings[0]  = digitalRead(sensor1); // Extreme Left
  irReadings[1]  = digitalRead(sensor2);
  irReadings[2]  = digitalRead(sensor3);
  irReadings[3]  = digitalRead(sensor4);
  irReadings[4]  = digitalRead(sensor5);
  irReadings[5]  = digitalRead(sensor6);
  irReadings[6]  = digitalRead(sensor7); //Extreme Right
}



//____________________________________________              _______________________________________________
//____________________________________________    Motors    _______________________________________________
//____________________________________________              _______________________________________________


void run_forward_1() {
  analogWrite(motor1pwmPin,  motor1newSpeed);
  digitalWrite(motor1Forward, HIGH);
  digitalWrite(motor1Backward,  LOW);
}
void run_forward_2() {
  analogWrite(motor2pwmPin,  motor2newSpeed);
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward,  HIGH);
}



//____________________________________________     PID     _______________________________________________
//____________________________________________    ERROR    _______________________________________________
//____________________________________________ CALCULATION _______________________________________________


void calculateError() {
  if ((irReadings[0] == Black) && (irReadings[1] ==  Black) && (irReadings[2] == Black) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == White)) {
    error = -6;
  } else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == White) && (irReadings[6] == White)) {
    error = -5;
  }  else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) &&  (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == White) && (irReadings[6] == Black)) {
    error = -4;
  } else if  ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) && (irReadings[3]  == Black) && (irReadings[4] == White) && (irReadings[5] == White) && (irReadings[6] == Black)) {
    error = -3;
  } else if ((irReadings[0]  == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) && (irReadings[3] == Black) &&  (irReadings[4] == White) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = -2;
  } else if ((irReadings[0] == Black) &&  (irReadings[1] == Black) && (irReadings[2] == Black) && (irReadings[3] == White) && (irReadings[4]  == White) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = -1;
  } else if ((irReadings[0] == Black) && (irReadings[1]  == Black) && (irReadings[2] == Black) && (irReadings[3] == White) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 0;
  } else if ((irReadings[0] == Black) && (irReadings[1] == Black) &&  (irReadings[2] == White) && (irReadings[3] == White) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error  =1;
  } else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2]  == White) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 2;
  } else if ((irReadings[0] == Black) && (irReadings[1] == White) && (irReadings[2]  == White) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 3;
  } else if ((irReadings[0] == Black) && (irReadings[1] == White) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 4;
  } else if ((irReadings[0] == White) && (irReadings[1] == White) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 5;
  } else if ((irReadings[0] == White) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    error = 6;
  }



  else if ((irReadings[0] == White) && (irReadings[1] == White) && (irReadings[2]  == White) && (irReadings[3] == White) && (irReadings[4] == White) && (irReadings[5] == White) && (irReadings[6] == White)) {
    error = 0;
  }
  /*
   else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black)) {
   error = ;}
*/

  else  if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) && (irReadings[3]  == Black) && (irReadings[4] == Black) && (irReadings[5] == Black) && (irReadings[6] == Black)) {
    if (previousError < 0)  {
      error = -50;
    } else if (previousError > 0) {
      error = 50;
    }
    else error = 0;
  }

}

//____________________________________________     PID     _______________________________________________
//____________________________________________    TERMS    _______________________________________________
//____________________________________________ CALCULATION _______________________________________________

void pidCalculations()  {

  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output  = pTerm + iTerm + dTerm;
  previousError = error;
}


//____________________________________________    PID     _______________________________________________
//____________________________________________   MOTORS   _______________________________________________
//____________________________________________   OUTPUT   _______________________________________________
void changeMotorSpeed()  {

  motor1newSpeed = motor1Speed  + OutputFactor * output;
  motor2newSpeed = motor2Speed - OutputFactor * output;

  if (motor2newSpeed > 200 )motor2newSpeed = 200;
  if (motor1newSpeed > 200) motor1newSpeed = 200;
  if (motor2newSpeed < 0) motor2newSpeed = 0;
  if (motor1newSpeed < 0) motor1newSpeed = 0;

  //Set new speed, and run motors in forward  direction
  run_forward_1();
  run_forward_2();

}


//____________________________________________     PID    _______________________________________________
//____________________________________________    MAIN    _______________________________________________
//____________________________________________   =BLOCK=  _______________________________________________

void Circulation() {
  //Put all of our functions here
  readIRSensors();
  calculateError();
  pidCalculations();
  changeMotorSpeed();
}

//____________________________________________                  _______________________________________________
//____________________________________________    STOP Motors   _______________________________________________
//____________________________________________                  _______________________________________________

void stopBot() {
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward,  LOW);
  digitalWrite(motor1Forward, LOW);
  digitalWrite(motor1Backward,  LOW);

}


//________________________________________________                      __________________________________________________________
//________________________________________________  No_PID_Circulation  __________________________________________________________
//________________________________________________                      __________________________________________________________

void forward(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, HIGH);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, HIGH);
}



void right(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed + 20); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, HIGH);
  digitalWrite(MOTOR_2_INPUT_1, HIGH);
  digitalWrite(MOTOR_2_INPUT_2, LOW);
}

void left(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed + 20); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, HIGH);
  digitalWrite(MOTOR_1_INPUT_2, LOW);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, HIGH);
}
void stopBot(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, LOW);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, LOW);
}

void loop() {
 Circulation();
 //forward(60);
}

