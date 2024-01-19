#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

double Kp = 0.5;
double Ki = 0.001;
double Kd = 0.001;
double lastError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double targetPosition = 2000.0;
double maxSpeed = 300.0;

void calibrateSensors()
{
  
  delay(1000);
  for(uint16_t i = 0; i < 60; i++)
  {
    if (i > 10 && i <= 50)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      delay(200);
      motors.setSpeeds(200, -200);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void lineFollowerPID(){

    ///brukt gpt til å finne hvilken type verdi, (eks. int, int16_t) som ville være best
    ///brukt gpt til å finne en ok startsverdi for konstantene til p, i og d. I tilegg til hvordan verdiene påvirker hverandre
  
  unsigned int lineSensorValues[5];
  int16_t position = lineSensors.readLine(lineSensorValues);
  int16_t error = position - targetPosition;
  integral += error;
  derivative = error - lastError;
  double output = Kp * error + Ki * integral + Kd * derivative;
  int16_t speedDifference = output;
  lastError = error;
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);

}

void setup()
{
  lineSensors.initFiveSensors();
  buttonA.waitForButton();
  calibrateSensors();

}

void loop()
{
  if(buttonA.getSingleDebouncedPress())
  {
    calibrateSensors();
  }
  lineFollowerPID();
}
