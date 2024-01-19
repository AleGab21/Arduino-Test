
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
int rute[] = {0,0,0,0,3,2,4};
int valgtRute = 0;
int chargerute[] = {0,1,5,0,3,2,4};



int acc = 0;
int lastPosition = 0;

double Kp = 0.5;
double Ki = 0.001;
double Kd = 0.001;
double lastError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double targetPosition = 2000.0;
double maxSpeed = 300.0;

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonB buttonC;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
unsigned int lineSensorValues[5];
Zumo32U4Motors motors;
Zumo32U4OLED display;
State state = pause_state;
unsigned long previousMillis = 0;
int motorSpeed = 250;
int leftCounter = 0;
String tekst[6] = {"", "", "", "", "", ""};

//////////////////////////////////////////////////////////
/////////////////////  Setup  ////////////////////////////
//////////////////////////////////////////////////////////

void setup()
{
  lineSensors.initFiveSensors();
  lineSensors.emittersOn();
  calibrateSensors();   

  Serial.begin(9600);
}

//////////////////////////////////////////////////////////
/////////////////////  Funksjons /////////////////////////
//////////////////////////////////////////////////////////

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

long tot_countsLeft = 0;
long tot_countsRight = 0; // definerer totalen for enkodertellern

void speedometer()
{
  static unsigned long lastDisplayTime;

  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {

    int intervall = 0;
    int displayTimer = 0;

    long countsLeft = encoders.getCountsAndResetLeft();
    long countsRight = encoders.getCountsAndResetRight(); // teller enkoderdataen og så resetter verdien ( for å ungå overflyt)

    tot_countsLeft += countsLeft;
    tot_countsRight += countsRight; // legger inn enkoderdataen i total variabelen

    double currentPosition = (tot_countsLeft + tot_countsRight) / 2; // finner gjennomsnittet av høyre og venstre side

    float deltaMillis = millis() - lastDisplayTime; // finner endring i tid

    float deltaPosition = (countsLeft + countsRight) / 2; // finner endringen i posisjon

    float speed = ((deltaPosition * 11250)) / (900 * deltaMillis); // finner fart ( ganger med faktorer, ut fra beregning for avtand per enkoder verdi)

    float distance = currentPosition * 11250 / 900000; // finner totale distansen kjørt med å bruke total verdiene

    /* display.setLayout11x4(); // setter opp display
    display.gotoXY(2, 0.4);
    display.print(speed); //speed
    display.gotoXY(1, 1);
    display.print(distance);// distance  */

    


    // switch case med variabel som endres med tidsintervaller
    // display gjennomsnittfart, toppfart.....

    lastPosition = currentPosition; // oppdaterer/nullstiller verdier
    lastDisplayTime = millis();
    countsLeft, countsRight = 0, 0;
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

void calibrateCar()
{

  bool calibrateButton = buttonB.isPressed(); // definerer knappetrykk

  if (calibrateButton)
  { // kalibreringsknapp
    calibrateSensors();
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
    
   if(valgtRute>=4)
   { 
    return lineSensorValues[0] == 1000 || lineSensorValues[4] ==1000 ||(lineSensorValues[0] < 200 && lineSensorValues[1] < 200 && lineSensorValues[2] < 200 && lineSensorValues[3] < 200 && lineSensorValues[4] < 200) ;
   }
   else
   {
    return lineSensorValues[0] == 1000 || lineSensorValues[4] ==1000;
   }
    
}

bool ignoreNoLine()
{
    return lineSensorValues[0] == 1000 || lineSensorValues[4] ==1000;
}

bool noLine()
{
    return (lineSensorValues[0] < 200 && lineSensorValues[1] < 200 && lineSensorValues[2] < 200 && lineSensorValues[3] < 200 && lineSensorValues[4] < 200);
}

void kryssAction()
{
    switch(rute[valgtRute])
    {
        case 0: //kjøre rett fram
        forward(100);
        valgtRute ++;
        break;
        case 1: //turn right
        motors.setSpeeds(0,0);
        turnRight(650);
        valgtRute ++;
        break;
        case 2: //turn left
        motors.setSpeeds(0,0);
        turnLeft(650);
        valgtRute ++;
        break;
        case 3: // reverse
        motors.setSpeeds(0,0);
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
        valgtRute ++;
        break;
        case 4:
        forward(500);
        valgtRute = 0;
        break;
        case 5:
        forward(500);
        delay(200);
        turnRight(400);
        valgtRute ++;
        break;



    }
}
void getToTheChargah()
{
    switch(chargerute[valgtRute])
    {
        case 0: //kjøre rett fram
        forward(100);
        valgtRute ++;
        break;
        case 1: //turn right
        motors.setSpeeds(0,0);
        turnRight(650);
        valgtRute ++;
        break;
        case 2: //turn left
        motors.setSpeeds(0,0);
        turnLeft(650);
        valgtRute ++;
        break;
        case 3: // reverse
        motors.setSpeeds(0,0);
        delay(200);
        backward(3000);
        delay(200);
        valgtRute ++;
        break;
        case 4:
        forward(500);
        valgtRute = 0;
        break;
        case 5:
        forward(500);
        delay(200);
        turnRight(400);
        valgtRute ++;
        break;
    }
}



//////////////////////////////////////////////////////////
/////////////////////  Loop  /////////////////////////////
//////////////////////////////////////////////////////////

void loop()
{

  speedometer();

  powerButton();


  calibrateCar();

  
    display.clear();
    display.gotoXY(1,1);
    display.print(valgtRute);
    display.gotoXY(0,0);
    display.print(state);

  switch (state)
  {
  case run_state:
    
    lineSensors.readCalibrated(lineSensorValues);
    while(!kryssDetect())
    {
        orgLineFollow();
    }
    state = kryss_state;

    break;

   case kryss_state:

      lineSensors.readCalibrated(lineSensorValues);
        if (kryssDetect() && batteryprosent < 20)
        {
            getToTheChargah();
        }
        
        else if(kryssDetect())
        {
            kryssAction();
        } 

        
        state = run_state;
        break;        
  default:
    break;
  }
}
