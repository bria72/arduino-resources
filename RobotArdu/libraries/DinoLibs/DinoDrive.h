#pragma once
#include <Adafruit_MotorShield.h>
#include <ArduinoBlue.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <SharpDistSensor.h>

class DinoDrive {
private:
    void Forward();
    void Forward(int time); 
    void Forward(int spd, int angle);
    void Backward();
    void Backward(int time); 
    void Backward(int spd, int angle);    int pos;
    Servo neck_servo;
    const uint8_t BLUETOOTH_TX;
    const uint8_t BLUETOOTH_RX;
    int MAX_SPEED;
    int STOP_SPEED;
    int MAX_PERCENT;
    int STOP_PERCENT;
    int MIN_SERVO_POS;
    int MAX_SERVO_POS;
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *motorBR;
    Adafruit_DCMotor *motorBL;
    Adafruit_DCMotor *motorFR;
    Adafruit_DCMotor *motorFL;
    const byte sensorPin;
    const byte medianFilterWindowSize;
    SharpDistSensor sensor;
    SoftwareSerial softSerial;
    ArduinoBlue phone; 
    int JeopardyMelody[];
    int JeopardyNoteDurations[];


public:

    DinoDrive();
    void Steer(int left_speed, int right_speed, int direction);
    void Steer(int left_speed, int right_speed, int direction, int time);
    void Turn_Left();
    void Turn_Right();
    void Turn_Left(int speed);
    void Turn_Left(int speed, int time);
    void Turn_Right(int speed);
    void Turn_Right(int speed, int time);
    void Stop();
    void moveNeck(int angle);
    void drive(int spd, int direction);
    void drive(int spd, int direction, int time);
    //Method to play music
    void playJeopardy();    
    bool checkObstacle();

};
