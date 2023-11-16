#include <Wire.h>
#include <Zumo32U4.h>

//////////////////////////////////////////////////////////
/////////////////////  Variables ////////////////////////
//////////////////////////////////////////////////////////

enum State
{
    pause_state,
    run_state
};



const int blackColor = 700;
int acc = 0;
int lastPosition = 0; 

float value = 0;
float bankKonto = 0; 

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
unsigned int lineSensorValues[5];
Zumo32U4Motors motors;
Zumo32U4OLED display;
int16_t lastError = 0;
State state = pause_state;
unsigned long previousMillis = 0;
int motorSpeed = 300; 
String tekst[6] = {"Ready", "Set", "GO!", "DONE", ">w<", "Distance: "};

//////////////////////////////////////////////////////////
/////////////////////  Setup  ////////////////////////////
//////////////////////////////////////////////////////////

void setup()
{

    lineSensors.initFiveSensors();
    lineSensors.emittersOn();


    Serial.begin(9600);
}

//////////////////////////////////////////////////////////
/////////////////////  Funksjons /////////////////////////
//////////////////////////////////////////////////////////




void calibrateSensors()
{

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(500);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else if (i>90 && i<=92)
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
    tot_countsRight += countsRight; //legger inn enkoderdataen i total variabelen 

    double currentPosition = (tot_countsLeft+tot_countsRight)/2; //finner gjennomsnittet av høyre og venstre side

    float deltaMillis = millis() - lastDisplayTime; // finner endring i tid

    float deltaPosition = (countsLeft + countsRight)/2; // finner endringen i posisjon

    float speed = ((deltaPosition*11250))/ (900*deltaMillis); // finner fart ( ganger med faktorer, ut fra beregning for avtand per enkoder verdi)
    

    float distance = currentPosition*11250/900000; // finner totale distansen kjørt med å bruke total verdiene
    
      display.setLayout11x4(); // setter opp display 
      display.gotoXY(2, 0.4);
      display.print(speed);
      display.gotoXY(1, 1);
      display.print(distance);

    
    // switch case med variabel som endres med tidsintervaller
    //display gjennomsnittfart, toppfart..... 

    lastPosition = currentPosition;  // oppdaterer/nullstiller verdier
    lastDisplayTime = millis();
    countsLeft, countsRight = 0,0;

  }
}

void powerButton(){

    bool buttonPress = buttonA.getSingleDebouncedPress(); //deffinerer knappetrykk 

    if (buttonPress && state == pause_state) // på-knapp
    {
        state = run_state;

        buttonPress = 0;

        
    }

    if (buttonPress && state == run_state) // setter programmet på pause igjen
    {
        state = pause_state;

        motors.setSpeeds(0,0);
    }

}

void calibrateCar(){

    bool calibrateButton = buttonB.isPressed(); // definerer knappetrykk


    if(calibrateButton){ //kalibreringsknapp 
        calibrateSensors();     
     }

}

/*
void batteryState{
  avhenging av fast, avstand og tid 

  
}*/

//////////////////////////////////////////////////////////
/////////////////////  Loop  /////////////////////////////
//////////////////////////////////////////////////////////

void loop()
{

    
    int16_t position = lineSensors.readLine(lineSensorValues); //leser av sensorverdier 
   

    speedometer();

    powerButton();

    calibrateCar();
    

    
   
    

    if (state == run_state) //linjefølgings regulering  
    {   
      if(1800 < position < 2200){

        motors.setSpeeds(motorSpeed, motorSpeed);

       }

       if(position > 2200){

        motors.setSpeeds(motorSpeed*1.5,motorSpeed*0.5);
        
       }
       
       if(position < 1800){

        motors.setSpeeds(motorSpeed*0.5, motorSpeed*1.5);

       }

       if (position > 3000)
       {

        motors.setSpeeds(motorSpeed*1.8, motorSpeed * 0.2);
        
       }

       if (position < 1000)
       {

        motors.setSpeeds(motorSpeed*0.2, motorSpeed*1.8);

       }
      
       
       
    
    }
}
