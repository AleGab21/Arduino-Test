
#include <IRremote.h>
#include <Wire.h>
#include <Zumo32U4.h>

//////////////////////////////////////////////////////////
/////////////////////  Variables ////////////////////////
//////////////////////////////////////////////////////////

enum State // setter opp de forskjellige statene til swtich-case, med bruk av enum, får ordene tilført en verdi fra 0, 1,2 osv.
{
  pause_state,
  run_state,
  circle_state,
  square_state,
  backforth_state,
  ski_state,
  calibrate_state,
  forward_state,
  backward_state,
  turnRight_state,
  turnLeft_state,
  Ok_state,
  PID_state,
};

int acc = 0;
int lastPosition = 0;

// pid variabler
double Kp = 0.5;
double Ki = 0.001;
double Kd = 0.001;
double lastError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double targetPosition = 2000.0;
double maxSpeed = 300.0;

// IR pins
const int RECV_PIN = A4;
IRrecv irrecv(RECV_PIN);

// ir controller buttons
long button1 = 3910598400;
long button2 = 3860463360;
long button3 = 4061003520;
long button4 = 4077715200;
long button5 = 3877175040;
long button6 = 2707357440;
long button7 = 4144561920;
long button8 = 3810328320;
long button9 = 2774204160;
long buttonStar = 3175284480;
long buttonHash = 3041591040;
long buttonUp = 3108437760;
long buttonRight = 3158572800;
long buttonLeft = 3141861120;
long buttonDown = 3927310080;
long buttonOk = 3208707840;

// kryss styring variabler
int i = 0;
long correct_array[] = {buttonUp, buttonDown, buttonLeft, buttonRight, button1, button2};
int user_array_len = 0;
long irNum = 0;

// defineringer og setting av generelle variabler og lignende
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
String tekst[6] = {"", "", "", "", "", ""}; // fikk ikke tid til å sette opp et byttende display, men skulle gjøres va bruk av denne eller et lignende array

//////////////////////////////////////////////////////////
/////////////////////  Setup  ////////////////////////////
//////////////////////////////////////////////////////////

void setup()
{
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  lineSensors.initFiveSensors();
  lineSensors.emittersOn();

  Serial.begin(9600);
}

//////////////////////////////////////////////////////////
/////////////////////  Funksjons /////////////////////////
//////////////////////////////////////////////////////////

void lineFollowerPID()
{

  /// brukt gpt til å finne hvilken type verdi, (eks. int, int16_t) som ville være best
  /// brukt gpt til å finne en ok startsverdi for konstantene til p, i og d. I tilegg til hvordan verdiene påvirker hverandre

  unsigned int lineSensorValues[5];                             // definerer linesensorene
  int16_t position = lineSensors.readLine(lineSensorValues);    // leseer av posisjonen på linjen fra 0-4000, samme konsept som brukt på den vanlige linjefølgeren
  int16_t error = position - targetPosition;                    // ser hvor langt fra ønsket plassering(midten) linjen er
  integral += error;                                            // legger til hvor stor forkjellen er til forje gang du sjekket
  derivative = error - lastError;                               // definerer den deriverte som forskjellen fra nye og gamle error verdien
  double output = Kp * error + Ki * integral + Kd * derivative; // ganger p, i og d verdiene med konstatner for å tilpasse de bilens styring
  int16_t speedDifference = output;                             // de sammenlagte verdiene defineres som speed diffirence og endres til 16 bit ( er denne typen ting vi trengte chat GTP til, siden hvilken størrelse de skulle være var vanskelig å se selv)
  lastError = error;
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference; // endrer hastigheten avhengig av forksjellene, lik måten brukt i orgLinjefølger hvor det er faktorer
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed); // contrainer verdiene, for å unngå at de blir større eller mindre enn ønsket verdier
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed); // setter til slutt hastighetene til motorene
}

void IRController()
{
  if (IrReceiver.decode()) // den overordnede if- setningen er den som mottar IR signalet, definerer det som irnum så resettes for å få nytt
  {
    unsigned long irNum = IrReceiver.decodedIRData.decodedRawData;

    Serial.println(irNum);

    if (irNum == button1) // resterende if setninger er bare definering av knapp verdier til å kjøre forskjellige states
    {
      state = run_state;
    }
    if (irNum == button2)
    {
      state = square_state;
    }
    if (irNum == button3)
    {
      state = backforth_state;
    }
    if (irNum == button4)
    {
      state = circle_state;
    }
    if (irNum == button5)
    {
      state = ski_state;
    }
    if (irNum == button6)
    {
      state = PID_state;
    }

    if (irNum == button8)
    {
      state = calibrate_state;
    }
    if (irNum == buttonUp)
    {
      state = forward_state;
    }
    if (irNum == buttonDown)
    {
      state = backward_state;
    }
    if (irNum == buttonRight)
    {
      state = turnRight_state;
    }
    if (irNum == buttonLeft)
    {
      state = turnLeft_state;
    }

    if (irNum == buttonStar)
    {
      state = pause_state;
    }

    if (irNum == buttonOk)
    {
      state = Ok_state;
    }
  }
  IrReceiver.resume();
}

void calibrateSensors()
{

  /* vent et halvt sekund så kalibrer sensorene
  med å rotere over linjen med sensorene
  dette er hovedsaklig fra eksempelkode, med lagt til delay i midten
  for å slite mindre på motorene*/
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

    display.setLayout11x4(); // setter opp display
    display.gotoXY(2, 0.4);
    display.print(speed);
    display.gotoXY(1, 1);
    display.print(distance);

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

  // ganger motorverdien med faktorer avhengig av avstanden linjen har fra midten av bilen
  // bortsett fra en bestem granseverdi med midten av bilen, for å unngå over korigering

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
  // en kode for kjøring bestemt av enkoderverdien du setter, bare brukt venstre side, siden å bruke begge virket overflødig for disse
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
  // Funskjon for svinging til høyre, bestemtemt av enkoderverdien du setter
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
  // samme som forje men for venstre, så siden den baserer seg på negativ motorspeed, at enkoder verdiene måtte bli -= istedet
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
  // samme som turn left bare for rygging, så nå at det kunne vært greit å bruke begge
  //  siden da kunne jeg brukt samme koden til alt,, selv om det ville blitt noen ekstra linjer
  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(-motorSpeed, -motorSpeed);
  while (countsLeft < count)
  {
    countsLeft -= encoders.getCountsAndResetLeft();
  };
}

void squareRun()
{
  // bruker kodene for kjøring jeg lagde og setter
  // dem i en for-løkke for å få den til å kjøre i en firkant
  // dette siden en firkant er bare en rekke med kjøringer fram og svinging samme vei

  for (int i = 0; i < 4; i++)
  {

    forward(5000);

    delay(500);

    turnRight(650);
  }
}

void backForthRun()
{

  // For bilen til å kjøre fram, så bruker turnRight med
  // dobblet verdi av det jeg fant til p være 90 grader
  // for å snu, så kjøre tilbake

  forward(5000);
  delay(500);
  turnRight(1300);
  delay(500);
  forward(5000);
}

void circleRun(long count)
{

  /*siden ingen av kodene jeg hadde laget kunne lett anvendes til en sirkel,
  Så brukte samme konseptet, men med inspirasjon for linjefølgingen, til å sette motorene til
  å svinge med en konstant faktor*/

  encoders.getCountsAndResetLeft();

  long countsLeft = 0;

  motors.setSpeeds(0.8 * motorSpeed, 1.2 * motorSpeed);
  while (countsLeft < count)
  {
    countsLeft += encoders.getCountsAndResetLeft();
  }

  motors.setSpeeds(0, 0);
}

void skiRun(long count)
{
  /*Prøvde å løse slalåm prugrammet med å kjøre i 2 halvsirkler,
  men i praksis funket det ikke, så måtte modifisere koden for sirkel en del
  for å få det til å kjøre i sikksakk med gjevne avstander*/

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  long countsLeft = 0;
  long countsRight = 0;
  motors.setSpeeds(0.8 * motorSpeed, 1.2 * motorSpeed); // første halvsirkel
  while (((countsLeft + countsRight) / 2) < count)
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();
  }

  forward(4000); // kjører litt fram for å få tilnermet lik startpos
  countsLeft = 0;
  countsRight = 0;

  motors.setSpeeds(1.2 * motorSpeed, 0.8 * motorSpeed);

  while (((countsLeft + countsRight) / 2) < count * 1.2) // andre, mer tilpasset halvsirkel
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();
  }

  forward(2500);

  motors.setSpeeds(0, 0);
}

//////////////////////////////////////////////////////////
/////////////////////  Loop  /////////////////////////////
//////////////////////////////////////////////////////////

void loop()
{

  speedometer(); // starter speedometer

  powerButton(); // aktiverer knappene på bilen hvis behov for å bruke dem

  IRController(); // aktiverer ir kontroller

  calibrateCar(); // setter opp kalibreringen

  /* display.clear();
  display.gotoXY(0,0);
  display.print(state); //setter displayet til å vise hvilken state vi er i, men var mer for testing
*/

  switch (state)
  {
  case run_state: // setter knapp 1 til å kjøre orginal linjefølger
    orgLineFollow();
    break;
  case square_state: // setter knapp 2 til å kjøre i en firkant
    delay(1000);
    squareRun();
    state = pause_state;
    break;
  case backforth_state: // setter knapp 3 til å kjøre fram og tilbake
    delay(1000);
    backForthRun();
    state = pause_state;
    break;
  case circle_state: // setter knapp 4 for å kjøre i en sirkel
    delay(1000);
    circleRun(11440);
    state = pause_state;
    break;
  case ski_state:
    delay(1000); // setter knapp 5 til å kjøre slalåm
    turnRight(650);
    skiRun(5720);
    state = pause_state;
    break;
  case calibrate_state: // 8(eight) to calibrate
    delay(500);
    calibrateSensors();
    state = pause_state;
    break;
  case forward_state: // setter opp pilen til å få bilen til å kjøre framover
    forward(800);
    state = pause_state;
    break;
  case backward_state: // setter ned pilen til å rygge
    backward(800);
    state = pause_state;
    break;
  case turnLeft_state: // siden kommandoen må gies forfra for å mota ir, så får jeg den til å svinge i hennhold til kotrolleren
    turnRight(300);    // venstre piltast vinger til høyre og omvendt
    state = pause_state;
    break;
  case turnRight_state:
    turnLeft(300);
    state = pause_state;
    break;
  case PID_state: // setter knapp 6 til å bruke PID linjefølgeren
    lineFollowerPID();
    break;
  /*case Ok_state:   // forsøk på "hemmelig oppladning, men hadde problemer med lagringen av knappetrykk"

    // denne skulle fungere med at vi trykker på ok knappen, som skulle starte lagringen av knappetrykk
    // deretter skulle disse verdiene sammenlignes med ett array, med en presett kode, som var up,down, left,right, 1,2
    // hvis disse var trykk i riktig rekkefølge, skulle du starte hemmelig lading
    // hvis du trykte feil rekkefølge eller ok på nytt, så hopper du ut av casen

    Serial.println("ok_state");
    Serial.println(irNum);
    if (user_array_len >= 5)
    {
      user_array_len = 0;
      //funksjon
      Serial.println("funksjonen kjører");
      state = pause_state;
    }

    if(correct_array[user_array_len]== irNum)
    {
      user_array_len++;
      Serial.println("len + 1");
    }
    else
    {
      user_array_len = 0;
      state = pause_state;
      Serial.println("len = 0");
    }

    break;  */
  default: // default som ingenting, bruker pause_state for å komme til den
    break;
  }
}
