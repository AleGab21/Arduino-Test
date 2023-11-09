//  Hva gjør denne koden?
//  1. Analyser  2. Kommenter  3. Skriv funksjonalitetskrav

/*
- Ventetid indikeres med rød RGB og begge led er slukket.
- Hver spiller har en bryter og en LED for å indikere når man har tapt eller vunnet.
- I tillegg er det en RGB led for å indikere spillets gang, og en buzzer for å indikere spillets gang.
- Spillet starter når ingen brytere er trykket.

- Spillet har en tilfeldig ventetid mellom 3 og 7 sekund.

- Om en bryter blir trykt før den tilfeldige ventetiden er over taper den spilleren.
	- Dette blir indikert av buzzer med konstant tone, rød RGB og lyser opp 
      spiller-LED for å indikere taper.
      
- Om ventetiden får gå ferdig uten knappetrykk vil RGB endre ril en grønnere farge, 
  og første knapp som blir trykket vinner.
	- Dette blir indikert av buzzer med stigende tone, grønn RGB og og lyser opp 
      spiller-LED for å indikere vinner.

- Når noen har vunnet eller tapt vil spillet automatisk starte på nytt.*/


int redLED = 10;     
int greenLED = 9;    
int buzzerPin = 11;

int LED1 = 6;
int LED2 = 7;
int SW1 = 3;	
int SW2 = 4;	

int winner = 0;        
int winnerBeep = 750;   
int fault = 0;          
int faultBeep = 200;    

unsigned long wait = 0;
unsigned long now = 0;

void setup(){
  pinMode(redLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(SW1,INPUT);
  pinMode(SW2,INPUT);
}

void prepareValues(){
  
  digitalWrite(redLED,HIGH);  

  fault = 0;
  now = millis(); // definerer tiden nå, som millis
  wait = now + random(3000,7000); // definerer en vente tid som er tilfeldig mellom 3-7 sekunder og avhengig av tiden når du starter
  
}

void waitToPress(){

/* setter at hvis noen trykker før tiden får du en fault og vil
da ikke gå inn i if løkken, men heller gå til else, hvor du får
buzzeren vil gi deg fault lyden*/

    while(millis() < wait && digitalRead(SW1)==HIGH && digitalRead(SW2)==HIGH){
  }
  if(digitalRead(SW1)==LOW)fault = LED1;
  if(digitalRead(SW2)==LOW)fault = LED2;
  digitalWrite(redLED,LOW); 

}

void runGame(){

 /* går inn i denne hvis du ikke har fått en fault, 
 med eks. feiltrykk den tilfeldige wait tiden må også ha være
 over for å gå ut av while løkken først*/

    if(fault == 0){ 
    digitalWrite(greenLED,HIGH);  //setter lyset til grønt
    while(digitalRead(SW1)==HIGH && digitalRead(SW2)==HIGH){ 
    } // venter til en av knappene er trykkt
    if(digitalRead(SW1)==LOW){ //bestemmer vinneren ut fra hvem som trykker først
      winner = LED1;
    }
    else{
      winner = LED2;
    }
    
    for(int k = 0; k < 5; k++){ 
      /*buzzer og lys for vinn*/
      tone(buzzerPin,(winnerBeep+(k*20))); 
      digitalWrite(greenLED,HIGH);
      digitalWrite(winner,HIGH);
      delay(50);
      digitalWrite(winner,LOW);
      digitalWrite(greenLED,LOW);
      delay(50); 
    }
    noTone(buzzerPin); // nullstiller lyden på buzzer
  }
  
  else{ /* buzzer og lys for fault*/
    tone(buzzerPin,faultBeep,500);
    for(int k = 0; k < 10; k++){
      digitalWrite(redLED,HIGH);
      digitalWrite(fault,HIGH);
      delay(50);
      digitalWrite(redLED,LOW);
      digitalWrite(fault,LOW);
      delay(50); 
    }
  }

}

void loop(){
  
  prepareValues();
  
  waitToPress();  

  runGame();


}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     Endringer    ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

/* La først alle delene hoveddelene av loopen inn i egne funksjoner,
laget mer konkret kommentering av koden,
*/