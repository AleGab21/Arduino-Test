#include <IRremote.hpp>
#include <Wire.h>
#include <Zumo32U4.h>

//////////////////////////////////////////////////////////
/////////////////////  Variables ////////////////////////
//////////////////////////////////////////////////////////

enum State
{
  pause_state,
  run_state,
  kryss_state
};

// turning globals
int rute[] = {0, 0, 0, 0, 3, 2, 4};
int valgtRute = 0;
int chargerute[] = {0, 1, 5, 0, 3, 2, 4};

// IR globals
const int RECV_PIN = A4;
IRrecv irrecv(RECV_PIN);
long irNum = 0;

float speed = 0;
float batteryLevel = 100;
int batteryLevelWatch = 0;
float batteryHealth = 100;
int batteryHealthWatch = 0;
float batteryDrain = 0;
int boost = 0;
int cState = 0;
int PROXIMITY = 0;
int BrakesON = 0;
unsigned long lastMillis = 0;
unsigned long lastMillisIR = 0;
unsigned long lastMillisBat = 0;
unsigned long lastMillisSolar = 0;
unsigned long lastMillisCarStop = 0;
long lastPos = 0;
float lastPosBat = 0;
long travel = 0;
const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
long totCountsLeft = 0;
long totCountsRight = 0;

Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonB buttonC;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
unsigned int lineSensorValues[5];
int lastError = 0;
Zumo32U4Motors motors;
Zumo32U4OLED display;
State state = pause_state;
unsigned long previousMillis = 0;
int motorSpeed = 250;
int leftCounter = 0;

//////////////////////////////////////////////////////////
/////////////////////  Setup  ////////////////////////////
//////////////////////////////////////////////////////////

void setup()
{
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
  lineSensors.initFiveSensors();
  lineSensors.emittersOn();
  calibrateSensors();

  Serial.begin(9600);
}

//////////////////////////////////////////////////////////
/////////////////////  Funksjons /////////////////////////
//////////////////////////////////////////////////////////

///////////////////////// Sensor /////////////////////////////

void IRController()
{

  if (IrReceiver.decode())
  {
    irNum = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(irNum);
  }
  IrReceiver.resume();
}

void calibrateSensors()
{

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(500);
  for (uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else if (i > 90 && i <= 92)
    {
      delay(250);
    }

    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

///////////////////////// Battery ////////////////////////////

void chargeBoost()
{
  int cState = buttonB.getSingleDebouncedPress();
  if (cState)
  {
    boost += 1;
    delay(1000);
  }

  if (boost == 1)
  {
    if (batteryLevel < 20)
    {
      motors.setSpeeds(-100, -100);
    }
  }

  if (boost == 1 and batteryLevel >= 20)
  {
    boost += 1;
  }
}

void batterydependentRUN()
{
  if (batteryLevel > 20)
  {
    batteryLevelWatch = 1;
  }
  if (batteryLevel <= 20 and batteryLevel > 0)
  {
    batteryLevelWatch = 2;
  }
  if (batteryLevel <= 0)
  {
    batteryLevelWatch = 3;
  }

  switch (batteryLevelWatch)
  {
  case 1:
    (motorSpeed = motorSpeed);
    break;

  case 2:
    (motorSpeed = 100);
    break;

  case 3:
    (motorSpeed = 0);
    break;
  }
}

void tooClose()
{
  proxSensors.read();
  int proxValLeft = proxSensors.countsFrontWithLeftLeds();
  int proxValRight = proxSensors.countsFrontWithRightLeds();

  PROXIMITY = (proxValLeft + proxValRight) / 2;
}

void calculations()
{
  totCountsLeft += encoders.getCountsAndResetLeft();
  totCountsRight += encoders.getCountsAndResetRight();

  travel = abs(((totCountsLeft + totCountsRight) / 2) / 7.9 / 10);

  unsigned long currentMillis = millis();
  float currentPos = (totCountsLeft + totCountsRight) / 2;
  if (currentMillis - lastMillis >= 100)
  {
    speed = abs((((currentPos - lastPos) / 7.9) / (currentMillis - lastMillis)) * 100);
    if (batteryLevel > 0 and speed > 20)
    {
      batteryLevel -= 0.01 + (currentPos - lastPos) / 100000 * speed;
      batteryHealth -= abs((0.01 + (currentPos - lastPos) / 100000 * speed) / 100);
    }

    if (batteryLevel > 0 and speed < 20)
    {
      batteryLevel -= 0.01 + (currentPos - lastPos) / 1000000 * speed;
      batteryHealth -= abs((0.01 + (currentPos - lastPos) / 1000000 * speed) / 10);
    }

    if (batteryLevel < 20 and ((currentPos - lastPos) < 0))
    {
      batteryLevel -= 0.01 + 10 * ((currentPos - lastPos) / 10000 * speed);
    }

    lastMillis = currentMillis;
    lastPos = currentPos;
    // EEPROM.write(0, batteryHealth);
  }
}

///////////////////////// Charge station ////////////////////////////

void ChargeStop()
{
  if (irNum == 1253111734)
  {
    BrakesON = 1;
  }
  if (irNum == 343317)
  {
    BrakesON = 0;
  }
}

void SolarCharging()
{
  unsigned long currentMillisSolar = millis();
  if (currentMillisSolar - lastMillisSolar >= 100)
  {
    if (irNum == -334604889 && batteryLevel < 100)
    {
      batteryLevel = batteryLevel + 0.5;
    }
    lastMillisSolar = currentMillisSolar;
  }
}

void OLED()
{
  int16_t position = lineSensors.readLine(lineSensorValues);
  static uint8_t lastDisplayTime;

  if ((uint8_t)(millis() - lastDisplayTime) >= 200)
  {
    lastDisplayTime = millis();
    display.clear();
    display.setLayout21x8();
    display.gotoXY(0, 0);
    display.print(travel);
    display.gotoXY(5, 0);
    display.print("cm");

    display.gotoXY(0, 2);
    display.print(speed);
    display.gotoXY(5, 2);
    display.print("cm/s");

    display.gotoXY(0, 4);
    display.print(round(batteryLevel), 1);
    display.gotoXY(5, 4);
    display.print("%");

    display.gotoXY(0, 6);
    display.print(irNum);
    /*     display.gotoXY(0, 6);
        display.print(round(batteryHealth), 1);
        display.gotoXY(5, 6);
        display.print("%"); */
  }
}

void powerButton()
{

  bool buttonPress = buttonA.getSingleDebouncedPress(); // deffinerer knappetrykk

  if (buttonPress && state == pause_state) // på-knapp
  {
    state = run_state;

    buttonPress = 0;
  }

  if (buttonPress && state != pause_state) // setter programmet på pause igjen
  {
    state = pause_state;

    motors.setSpeeds(0, 0);
  }
}

void orgLineFollow()
{

  int16_t position = lineSensors.readLine(lineSensorValues); // leser av sensorverdier

  if (1800 < position < 2200)
  {

    motors.setSpeeds(motorSpeed, motorSpeed);
  }

  if (position > 2200)
  {

    motors.setSpeeds(motorSpeed * 1.5, motorSpeed * 0.5);
  }

  if (position < 1800)
  {

    motors.setSpeeds(motorSpeed * 0.5, motorSpeed * 1.5);
  }

  if (position > 3000)
  {

    motors.setSpeeds(motorSpeed * 1.8, motorSpeed * 0.2);
  }

  if (position < 1000)
  {

    motors.setSpeeds(motorSpeed * 0.2, motorSpeed * 1.8);
  }
}

void forward(long count)
{
  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(motorSpeed, motorSpeed);
  while (countsLeft < count)
  {
    countsLeft += encoders.getCountsAndResetLeft();
  };

  motors.setSpeeds(0, 0);
}

void turnRight(long count)
{
  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(motorSpeed, -motorSpeed);
  while (countsLeft < count)
  {
    countsLeft += encoders.getCountsAndResetLeft();
  };

  motors.setSpeeds(0, 0);
}

void turnLeft(long count)
{
  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(-motorSpeed, motorSpeed);
  while (countsLeft < count)
  {
    countsLeft -= encoders.getCountsAndResetLeft();
  };

  motors.setSpeeds(0, 0);
}

void backward(long count)
{
  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(-motorSpeed, -motorSpeed);
  while (countsLeft < count)
  {
    countsLeft -= encoders.getCountsAndResetLeft();
  };
}

bool kryssDetect()
{

  if (valgtRute >= 4)
  {
    return lineSensorValues[0] == 1000 || lineSensorValues[4] == 1000 || (lineSensorValues[0] < 200 && lineSensorValues[1] < 200 && lineSensorValues[2] < 200 && lineSensorValues[3] < 200 && lineSensorValues[4] < 200);
  }
  else
  {
    return lineSensorValues[0] == 1000 || lineSensorValues[4] == 1000;
  }
}

bool ignoreNoLine()
{
  return lineSensorValues[0] == 1000 || lineSensorValues[4] == 1000;
}

bool noLine()
{
  return (lineSensorValues[0] < 200 && lineSensorValues[1] < 200 && lineSensorValues[2] < 200 && lineSensorValues[3] < 200 && lineSensorValues[4] < 200);
}

void kryssAction()
{
  switch (rute[valgtRute])
  {
  case 0: // kjøre rett fram
    forward(100);
    valgtRute++;
    break;
  case 1: // turn right
    motors.setSpeeds(0, 0);
    turnRight(650);
    valgtRute++;
    break;
  case 2: // turn left
    motors.setSpeeds(0, 0);
    turnLeft(650);
    valgtRute++;
    break;
  case 3: // reverse
    motors.setSpeeds(0, 0);
    delay(200);
    backward(3000);
    delay(200);

    /* while (noLine())
     {
         noLine();
         backward(100);
         delay(100);
     }
     while (!ignoreNoLine())
     {
         ignoreNoLine();
         backward(100);
         delay(100);
     }  */
    valgtRute++;
    break;
  case 4:
    forward(500);
    valgtRute = 0;
    break;
  case 5:
    forward(500);
    delay(200);
    turnRight(400);
    valgtRute++;
    break;
  }
}

void getToTheChargah()
{
  switch (chargerute[valgtRute])
  {
  case 0: // kjøre rett fram
    forward(100);
    valgtRute++;
    break;
  case 1: // turn right
    motors.setSpeeds(0, 0);
    turnRight(650);
    valgtRute++;
    break;
  case 2: // turn left
    motors.setSpeeds(0, 0);
    turnLeft(650);
    valgtRute++;
    break;
  case 3: // reverse
    motors.setSpeeds(0, 0);
    delay(200);
    backward(3000);
    delay(200);
    valgtRute++;
    break;
  case 4:
    forward(500);
    valgtRute = 0;
    break;
  case 5:
    forward(500);
    delay(200);
    turnRight(400);
    valgtRute++;
    break;
  }
}

void mainSwitch()
{
  if (BrakesON == 0 && PROXIMITY < 5)
  {
    switch (state)
    {
    case run_state:

      lineSensors.readCalibrated(lineSensorValues);
      while (!kryssDetect() )
      {
        OLED();
        orgLineFollow();
      }
      state = kryss_state;

      break;

    case kryss_state:

      lineSensors.readCalibrated(lineSensorValues);
      if (kryssDetect() && batteryLevel < 20)
      {
        getToTheChargah();
      }

      else if (kryssDetect())
      {
        kryssAction();
      }

      state = run_state;
      break;
    default:
      break;
    }
  }
  if (BrakesON == 1 || PROXIMITY >= 5)
  {
    motors.setSpeeds(0, 0);
  }
}

//////////////////////////////////////////////////////////
/////////////////////  Loop  /////////////////////////////
//////////////////////////////////////////////////////////

void loop()
{
  OLED();
  ChargeStop();
  chargeBoost();
  calculations();
  SolarCharging();
  batterydependentRUN();
  IRController();
  powerButton();
  mainSwitch();
}
