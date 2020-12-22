#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <Servo.h>
#include "pitches.h"
#include <SharpDistSensor.h>
#include <DinoDrive.h>

DinoDrive::DinoDrive(): pos{ 10 }, BLUETOOTH_TX{ 12 }, BLUETOOTH_RX{ 11 }, MAX_SPEED{ 200 }, STOP_SPEED{ 0 }, MAX_PERCENT{ 100 }, STOP_PERCENT{ 0 }, MIN_SERVO_POS{ 0 }, MAX_SERVO_POS{ 55 }, AFMS{ Adafruit_MotorShield() }, motorBL{ AFMS.getMotor(1) }, motorFR{ AFMS.getMotor(3) }, motorFL{ AFMS.getMotor(4) }, motorBR{ AFMS.getMotor(2) }, sensorPin{ A2 }, medianFilterWindowSize{ 5 }, sensor{ sensorPin, medianFilterWindowSize }, softSerial{ BLUETOOTH_TX, BLUETOOTH_RX }, phone{ softSerial }, JeopardyMelody{
  NOTE_C2, NOTE_F3, NOTE_C3, NOTE_A2,
  NOTE_C3, NOTE_F3, NOTE_C3,
  NOTE_C3, NOTE_F3, NOTE_C3, NOTE_F3,
  NOTE_AS3, NOTE_G3, NOTE_F3, NOTE_E3, NOTE_D3, NOTE_CS3,
  NOTE_C3, NOTE_F3, NOTE_C3, NOTE_A2, 
  NOTE_C3, NOTE_F3, NOTE_C3,
  NOTE_AS3, 0, NOTE_G3, NOTE_F3,
  NOTE_E3, NOTE_D3, NOTE_CS3, NOTE_C3
}, JeopardyNoteDurations{
4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 3, 8, 8, 8, 8, 8,
4, 4, 4, 4, 4, 4, 2, 4, 8, 8, 4, 4, 4, 4, 4, 4, 0
}
{
}

/*
* Steer function allows assignment of different speed for left and right motors
* if direction = 1, moves forward; if direction = 0, moves backward
* calls overloaded steer function with time = 500ms
* @Params int left_speed, int right_speed, int direction
*/

void DinoDrive :: Steer(int left_speed, int right_speed, int direction) { // if 1 forward if 0 backward
  Steer(left_speed, right_speed, direction, 500);
}

/*
* Steer function overloaded, with additional time parameter(ms)
* @params int left_speed, int right_speed, int direction, int time
*/
void DinoDrive :: Steer(int left_speed, int right_speed, int direction, int time) { // if 1 forward if 0 backward
  left_speed = abs(left_speed);
  int leftThrottle = constrain(left_speed, 0, 100);
  leftThrottle = map(leftThrottle, STOP_PERCENT, MAX_PERCENT, STOP_SPEED, MAX_SPEED);

  right_speed = abs(right_speed);
  int rightThrottle = constrain(right_speed, 0, 100);
  rightThrottle = map(rightThrottle, STOP_PERCENT, MAX_PERCENT, STOP_SPEED, MAX_SPEED);

  motorBL->setSpeed(leftThrottle);
  motorBR->setSpeed(rightThrottle);
  motorFL->setSpeed(leftThrottle);
  motorFR->setSpeed(rightThrottle);

  if(direction == 0){
    Backward(time);
  }
  else{
    Forward(time);
  }
}

/*
* Turn_Left turns the dinobot left
* @params int speed
* calls overloaded Turn_Left method with time = 500ms
*/
void DinoDrive :: Turn_Left(int speed){              
    Turn_Left(speed, 500);  
}

/*
* Turn_Left turns the dinobot left
* @params int speed, int time
*/
void DinoDrive :: Turn_Left(int speed, int time){ //time in ms
    int mappedSpd = constrain(abs(speed), 0, 100);
    mappedSpd = map(mappedSpd, STOP_PERCENT, MAX_PERCENT, STOP_SPEED, MAX_SPEED);
 // setting motor speeds
    motorBL->setSpeed(speed);
    motorBR->setSpeed(speed);
    motorFL->setSpeed(speed);
    motorFR->setSpeed(speed);
    Serial.println("Turn Left with Speed and time");                
    motorBL->run(BACKWARD);
    motorBR->run(FORWARD);
    motorFL->run(FORWARD);
    motorFR->run(BACKWARD);       
    delay(time);      
    Serial.println("done");
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE);
}

/*
* Turns the dinobot right
* @params int speed
* calls overloaded Turn_Right method with time = 500ms
*/
void DinoDrive :: Turn_Right(int speed) {
    Turn_Right(speed, 500);
}

/*
* Turns the dinobot right
* @params int speed, int time
*/
void DinoDrive :: Turn_Right(int speed, int time) { //time in ms
  
    motorBL->setSpeed(speed);
    motorBR->setSpeed(speed);
    motorFL->setSpeed(speed);
    motorFR->setSpeed(speed);  
    Serial.println("Turn Right with speed");
    motorBL->run(FORWARD);
    motorBR->run(BACKWARD);
    motorFL->run(BACKWARD);
    motorFR->run(FORWARD);  
    delay(time);        
    Serial.println("done");
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE); 
}

/*
* moves dinobot in direct and speed chosen by user
* @params int spd, int direction
* calls overloaded drive method with time = 500ms
*/
void DinoDrive :: drive(int spd, int direction){
  drive(spd, direction, 500);
}

/*
* moves dinobot in direct and speed chosen by user
* @params int spd, int direction, int time
*/
void DinoDrive :: drive(int spd, int direction, int time){
  int mappedSpd;
  spd = abs(spd);
  mappedSpd = constrain(spd, 0, 100);
  mappedSpd = map(mappedSpd, STOP_PERCENT, MAX_PERCENT, STOP_SPEED, MAX_SPEED);

  
  motorBL->setSpeed(mappedSpd);
  motorBR->setSpeed(mappedSpd);
  motorFL->setSpeed(mappedSpd);
  motorFR->setSpeed(mappedSpd); 

  if(direction == 0){
    Backward(time); 
  }
  else{
    Forward(time);
  }
}

/*
* moves dinobot forward for a given time
* @params int time
*/
void DinoDrive :: Forward(int time) {
  Serial.println("Inside MOVEFORWARD()");  
  Serial.println("Forward");    

    //run the wheels in the forward direction            
    motorBL->run(FORWARD);
    motorBR->run(FORWARD);
    motorFL->run(BACKWARD);
    motorFR->run(BACKWARD);
    delay(time);
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE); 
}

/*
* moves dinobot backward for 500 ms by calling overloaded Backward() method
*/
void DinoDrive :: Backward() {
  Backward(500);
}

/*
* moves dinobot backward for a given time
* @params int time
*/
void DinoDrive :: Backward(int time) {
  Serial.println("Inside MOVEBACKWARDS()");
  Serial.println("Reverse");                

    motorBL->run(BACKWARD);
    motorBR->run(BACKWARD);
    motorFL->run(FORWARD);
    motorFR->run(FORWARD);
    delay(time);
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE); 
}

/*
* moves dinobot forward for 500 ms by calling overloaded Forward() method
*/
void DinoDrive :: Forward() {
  Forward(500);
}

/*
* moves dinobot forward for a given time and at an angle
* @params int time, int angle
*/
void DinoDrive :: Forward(int spd, int angle){
    int mappedSpd;
    
    mappedSpd = map(angle, 90, 0, spd, 0);
    if(angle > 90){ //right turn
        //set the speed found of each wheel
        motorBL->setSpeed(spd);
        motorBR->setSpeed(mappedSpd);
        motorFL->setSpeed(spd);
        motorFR->setSpeed(mappedSpd);    
    }else if(angle < 90){ //left turn
        //se t the speed found of each wheel
        motorBL->setSpeed(mappedSpd);
        motorBR->setSpeed(spd);
        motorFL->setSpeed(mappedSpd);
        motorFR->setSpeed(spd);
    }

    //Wheels go striaght
    Forward();
}

/*
* moves dinobot backward for a given time and at an angle
* @params int time, int angle
*/
void DinoDrive :: Backward(int spd, int angle){
    int mappedSpd;
    
    mappedSpd = map(angle, 90, 0, spd, 0);    
    if(angle > 90){ //right turn
        //set the speed found of each wheel
        motorBL->setSpeed(spd);
        motorBR->setSpeed(mappedSpd);
        motorFL->setSpeed(spd);
        motorFR->setSpeed(mappedSpd);    
    }else if(angle < 90){ //left turn
        //set the speed found of each wheel
        motorBL->setSpeed(mappedSpd);
        motorBR->setSpeed(spd);
        motorFL->setSpeed(mappedSpd);
        motorFR->setSpeed(spd);
    }
    
    //run wheels back
    Backward();
}

/*
* Turns the dinobot left for 500ms at default speed
*/
void DinoDrive :: Turn_Left() {
  Serial.println("Inside TURNLEFT()");
    
    Serial.println("Turn Left");                
    motorBL->run(BACKWARD);
    motorBR->run(FORWARD);
    motorFL->run(FORWARD);
    motorFR->run(BACKWARD);       
    delay(500);      
    Serial.println("done");
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE);
}

/*
* Turns the dinobot right for 500ms at default speed
*/
void DinoDrive :: Turn_Right() {
  Serial.println("Inside TURNRIGHT()");
    
    Serial.println("Turn Right");
    motorBL->run(FORWARD);
    motorBR->run(BACKWARD);
    motorFL->run(BACKWARD);
    motorFR->run(FORWARD);  
    delay(500);        
    Serial.println("done");
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE); 
}

/*
* stops all motors of dinobot
*/
void DinoDrive :: Stop() {
  Serial.println("Inside STOP()");
   delay(200);
    motorBL->run(RELEASE);
    motorBR->run(RELEASE);
    motorFL->run(RELEASE);
    motorFR->run(RELEASE); 
}

/*
* Moves the dinobots neck based on user input angle
* @params int angle
*/
void DinoDrive :: moveNeck(int angle){

  angle = constrain(angle, 0, 90);
  int mappedPos = map(angle, 0, 90, MIN_SERVO_POS, MAX_SERVO_POS);
  if(pos < mappedPos){//if slider is greater than current position so move neck up incrementally
          while(pos < mappedPos){
              pos += 2;
              neck_servo.write(pos);
              delay(15);
              Serial.println(pos);
          }
   }
   else if(pos > mappedPos){//slider is less than current position so move neck down incrementally
          while(pos > mappedPos){
              pos -= 2;
              neck_servo.write(pos);
              Serial.println(pos);
             delay(15);
        }
    }
}

/*
* plays jeapoardy
*/
void DinoDrive :: playJeopardy(){//Jeopardy Tone
  Serial.println("Inside PLAYJEOPARDY()");
     // iterate over the notes of the melody:
  for (int thisNote = 0; JeopardyNoteDurations[thisNote] != 0; thisNote++) {
    // to calculate the note duration, take one second divided by the note type ie. quarter note = 1000 / 4, eighth note = 1000/8, etc.
       int noteDuration = 2000/JeopardyNoteDurations[thisNote];
       tone(2, JeopardyMelody[thisNote],noteDuration * 0.9);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
      //int pauseBetweenNotes = noteDuration * 1.30;
      //delay(pauseBetweenNotes);
      delay(noteDuration);
  }
}


/*
* checks if there is an obstacle within 110 (unit) of dinobot
* @return boolean value; if obstacle present returns true 
*/
bool DinoDrive :: checkObstacle(){
// Get distance from sensor and print
  unsigned int distance = sensor.getDist();
  Serial.println("Distance:  ");
  Serial.println(distance);
  delay(50);

  if(distance < 110){
 //     Stop();
      return true;  
  }
  
return false;
}
