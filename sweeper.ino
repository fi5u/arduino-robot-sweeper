// Ref: https://github.com/mjs513/MPU6050-Motion-Detection/blob/master/MPU6050_MotionDetect.ino

#include <Wire.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include "MPU6050_6Axis_MotionApps20.h"

MotorDriver motor(0); //value passed is the address- remove resistor R1 for 1, R2 for 2, R1 and R2 for 3
MotorDriver motor1(1);

// Accelerometer
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int8_t threshold, count;
float temp;
bool zero_detect;
bool TurnOnZI = true;
bool XnegMD, XposMD, YnegMD, YposMD, ZnegMD, ZposMD;
#define LED_PIN 13
bool blinkState = false;

// Config
const bool testMode = false;

// Motor
const int maxPWM = 10000;
const int steps = 100;
const int stepSize = maxPWM/steps;
const int velPWM = 6000;
bool isTurning = false;

// Ping
const byte sonarNum = 3; // number of sensors
const byte triggerPinLeft = 6;
const byte echoPinLeft = 5;
const byte triggerPinCenter = 3;
const byte echoPinCenter = 7;
const byte triggerPinRight = 8;
const byte echoPinRight = 4;
const int maxDistance = 200;
const byte detectionDistCM = 6;
bool ignoreObstacles = false;
const byte falsePositiveLimit = 3; // cycles of is obstacle before reacting
byte falsePositiveCount = 0;

// Stuck
const float zeroMoveMaxSecs = 1.2;
unsigned long motionStartTime = millis();


NewPing sonar[sonarNum] = {
    NewPing(triggerPinLeft, echoPinLeft, maxDistance),
    NewPing(triggerPinCenter, echoPinCenter, maxDistance),
    NewPing(triggerPinRight, echoPinRight, maxDistance)
};

void setup() {
    Serial.begin(9600);
    Wire.begin();

    Serial.println("=========");
    Serial.println("= START =");
    Serial.println("=========");

    // Accelerometer
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setAccelerometerPowerOnDelay(3);
    accelgyro.setIntZeroMotionEnabled(TurnOnZI);
    accelgyro.setDHPFMode(1);
    accelgyro.setMotionDetectionThreshold(1);
    accelgyro.setZeroMotionDetectionThreshold(1);
    accelgyro.setMotionDetectionDuration(5);
    accelgyro.setZeroMotionDetectionDuration(1);
    pinMode(LED_PIN, OUTPUT);

    //The value passed to begin() is the maximum PWM value, which is 16 bit(up to 65535)
    //This value also determines the output frequency- by default, 8MHz divided by the maxPWM value
    if(motor.begin(maxPWM)){
        Serial.println("Motor driver 0 not detected!");
        while(1);
    }
    if(motor1.begin(maxPWM)){
        Serial.println("Motor driver 1 not detected!");
        while(1);
    }

    //The failsafe turns off motors if a command is not sent in a certain amount of time.
    //Failsafe is set in milliseconds- comment or set to 0 to disable
    motor.setFailsafe(1000);
    motor1.setFailsafe(1000);
}

void drive(String dir = "forward") {
    if(testMode) { return; }

    if(dir == "forward") {
        motor.setMotor(1, velPWM);
        motor.setMotor(2, velPWM);
    }
    else if(dir == "reverse") {
        motor.setMotor(1, -velPWM);
        motor.setMotor(2, -velPWM);
    }
}

void turn(String dir = "clockwise", String turn = "rand") {
    const int turnQuarter = 4450;
    const int turnHalf = 8400;
    const int turnFull = 16000;

    const int m1 = dir == "clockwise" ? velPWM : - velPWM;
    const int m2 = dir == "clockwise" ? - velPWM : velPWM;

    int turnTime = 0;
    if(turn == "quarter") {
        turnTime = turnQuarter;
    }
    else if(turn == "half") {
        turnTime = turnHalf;
    } else {
        // Random
        turnTime = random(500, turnFull - 500);
    }

    unsigned long turnStartTime = millis();
    isTurning = true;
    ignoreObstacles = true;

    if(!testMode) {
        while(millis() - turnStartTime < turnTime) {
            motor.setMotor(1, m1);
            motor.setMotor(2, m2);
            Serial.println("Turning");
            Serial.println(millis() - turnStartTime);
        }
    }

    isTurning = false;
    ignoreObstacles = false;
}

bool fireSonar(int index, String label) {
    int uS = sonar[index].ping();
    float distanceCM = uS / US_ROUNDTRIP_CM;
    if(distanceCM > 0 && distanceCM < detectionDistCM) {
        Serial.println("Is obstacle " + label + " :");
        Serial.println(distanceCM);
        return true;
    }
    return false;
}

String obstacleDirection() {
    if(ignoreObstacles) { return "none"; }
    if(fireSonar(0, "left")) {
        Serial.println("Is obstacle LEFT");
        return "left";
    }
    if(fireSonar(1, "center")) {
        Serial.println("Is obstacle CENTER");
        return "center";
    }
    if(fireSonar(2, "right")) {
        Serial.println("Is obstacle RIGHT");
        return "right";
    }
    return "none";
}

void reverseAndTurn() {
    const byte reverseSeconds = 8;
    unsigned long startTime = millis();
    ignoreObstacles = true;
    Serial.println("Reversing and turning");
    while((millis() - startTime) / 1000 < reverseSeconds) {
        drive("reverse");
    }
    turn();
    ignoreObstacles = false;
}

bool checkMovement() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    zero_detect = accelgyro.getIntMotionStatus();

    if(zero_detect == 1) {
        motionStartTime = millis();
    }
}

bool isStuck() {
    if((millis() - motionStartTime) / 1000 > zeroMoveMaxSecs) {
        return true;
    }

    return false;
}

void sweep() {
    motor1.setMotor(1, 30000);
    motor1.setMotor(2, 30000);
}

void loop() {
    checkMovement();

    sweep();

    drive("forward");

    if(isStuck()) {
        digitalWrite(LED_PIN, HIGH);
        reverseAndTurn();
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    // Check the obstacle detection is not a false positive
    String obstacleDir = obstacleDirection();
    if(obstacleDir != "none" && !isTurning) {
        falsePositiveCount++;
        Serial.println("INCREASING FALSEPOSITIVE COUNT");
        if(falsePositiveCount >= falsePositiveLimit) {
            // Not a false positive, turn away from obstacle
            falsePositiveCount = 0;
            if(obstacleDir == "left") {
                turn("clockwise", "quarter");
            }
            else if(obstacleDir == "right") {
                turn("anticlockwise", "quarter");
            }
            else {
                turn();
            }
        }
    }
    else if(falsePositiveCount != 0) {
        // Obstacle detection was a false positive
        Serial.println("False positive count reset");
        falsePositiveCount = 0;
    }
}
