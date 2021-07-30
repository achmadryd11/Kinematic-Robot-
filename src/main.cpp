#include <Arduino.h>
#include <kinematics_RS.h>

// Motor Variable
#define MLF 9
#define MLB 10
#define PWM_ML 3

#define MRF 11
#define MRB 12
#define PWM_MR 2

// Encoder Variable
#define encoderLeftA 18
#define encoderLeftB 19

#define encoderRightA 20
#define encoderRightB 21

#define phi 3.14285714286
#define rangeBetweenWheels 18.5
#define wheelRadius 6.5
#define encoderPPR 12

//Millis Variable
unsigned long startTimeMillis,
              endTimeMillis,
              loopTimer;
    
kinematicAUMR_RS kinematics(encoderLeftA, encoderLeftB, encoderRightA, encoderRightB, wheelRadius, rangeBetweenWheels, encoderPPR );

//Kinematic Variable
int xPositionInCM,
    yPositionInCM,
    thetaPositionInDegree,
    leftLinearSpeed,
    rightLinearSpeed;

float leftPosition,
      rightPosition;

// Millis 
void checkLoopTimer()
{
  endTimeMillis = millis();
  loopTimer = (endTimeMillis - startTimeMillis);
  startTimeMillis = millis();

  Serial.print(loopTimer);
  Serial.print(" \t");
}

void getKinematicData()
{
  Serial.print(" LeftEnc: ");
  Serial.print(leftPosition);
  Serial.print(" RightEnc: ");
  Serial.print(rightPosition);
  Serial.print(" xPostCM: ");
  Serial.print(xPositionInCM);
  Serial.print(" yPostCM: ");
  Serial.print(yPositionInCM);
  Serial.print(" ThetPostDeg: ");
  Serial.print(thetaPositionInDegree);
  Serial.print(" LeftLinSpd: ");
  Serial.print(leftLinearSpeed);
  Serial.print(" RghtLnrSpd: ");
  Serial.println(rightLinearSpeed);
}

void processLeftForward()
{
  kinematics.processLeftForward();
}

void processLeftBackward()
{
  kinematics.processLeftBackward();
}

void processRightForward()
{
  kinematics.processRightForward();
}

void processRightBackward()
{
  kinematics.processRightBackward();
}

void setup() {
  Serial.begin(9600);
  Serial.println("CLEARDATA");
  Serial.println("LABEL,CLOCK,xPositionInCM,yPositionInCM");

  pinMode(MLF, OUTPUT);
  pinMode(MLB, OUTPUT);
  pinMode(MRF, OUTPUT);
  pinMode(MLF, OUTPUT);
  pinMode(PWM_ML, OUTPUT);
  pinMode(PWM_MR, OUTPUT);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), processLeftForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLeftB), processLeftBackward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), processRightForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightB), processRightBackward, CHANGE);
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  checkLoopTimer();

  kinematics.calculate();
  yPositionInCM = kinematics.getYPositionInCM();
  xPositionInCM = kinematics.getXPositionInCM();
  thetaPositionInDegree = kinematics.getThetaInDegree();
  leftLinearSpeed = kinematics.getLeftSpeed();
  rightLinearSpeed = kinematics.getRightSpeed();
  leftPosition = kinematics.getLeftPositionInCM();
  rightPosition = kinematics.getRightPositionInCM();

  //Print Kinematic Data to PC
  getKinematicData();

}