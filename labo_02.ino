#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0
MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sensorDistance(PORT_10);
enum AppState {
  BOOT,
  MARCHE,
  LENT,
  ARRET,
  RECULE,
  PIVOTE
};

AppState etat = BOOT;


int maxPwm = 255;
int halfPwm = 125;
int turnPwm = 150;
int croisiere = maxPwm * 0.60;
int lenteur = croisiere * 0.50;
//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 48;  // M1 ENA
const int m1_in2 = 49;  // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47;  // M2 ENA
const int m2_in2 = 46;  // M2 ENB


unsigned long currentTime = 0;
double distance;
// Les pragma region permettent de créer des zones de code fermable
#pragma region Modèles
void mesurerDistance(unsigned long cT) {
  const int rate = 100;
  static unsigned long lastTime = 0;
  if (cT - lastTime < rate) return;

  lastTime = cT;
  distance = constrain(sensorDistance.distanceCm(), 5, 300);
}

void lent(unsigned long cT) {
  static unsigned long lastTime = 0;


  static bool firstTime = true;
  //Set speed via PWM
  if (firstTime) {
    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, lenteur);  //Set speed via PWM

    digitalWrite(m2_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    analogWrite(m2_pwm, lenteur);
    led.setColor(0, 0, 0);
    for (int i = 0; i < 6; i++) {
      led.setColorAt(i, 255, 255, 0);
    }
    led.show();
    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état


  if (distance < 30) {

    etat = ARRET;
    firstTime = true;
  } else if (distance >= 100) {

    etat = MARCHE;
    firstTime = true;
  }

  // Il est possible d'avoir plusieurs transitions
}


void marche(unsigned long cT) {
  static unsigned long lastTime = 0;


  static bool firstTime = true;
  //Set speed via PWM
  if (firstTime) {
    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, croisiere);  //Set speed via PWM

    digitalWrite(m2_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    analogWrite(m2_pwm, croisiere);
    led.setColor(0, 0, 0);
    for (int i = 6; i < 12; i++) {
      led.setColorAt(i, 0, 255, 0);
    }
    led.show();
    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état


  bool transition = false;
  if (distance < 100) {
    transition = true;
  }
  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

    firstTime = true;
    etat = LENT;
    Serial.println("SORTIE : MARCHE");
    // appState = PROCHAIN_ETAT;
  }
}

void arret(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 2000;

  static bool firstTime = true;
  //Set speed via PWM
  if (firstTime) {

    analogWrite(m1_pwm, 0);  //Set speed via PWM

    analogWrite(m2_pwm, 0);

    led.setColor(255, 0, 0);

    led.show();
    lastTime = cT;
    firstTime = false;
    return;
  }
  if (cT - lastTime < rate) return;



  // Code de SORTIE
  // Code pour terminer l'état

  firstTime = true;
  etat = RECULE;
  // appState = PROCHAIN_ETAT;
}
void recule(unsigned long cT) {
  static unsigned long lastTime = 0;

  const int rate = 1000;
  static bool firstTime = true;
  //Set speed via PWM
  if (firstTime) {
    digitalWrite(m1_in2, HIGH);
    digitalWrite(m1_in1, LOW);
    analogWrite(m1_pwm, croisiere);  //Set speed via PWM

    digitalWrite(m2_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    analogWrite(m2_pwm, croisiere);




    led.show();
    lastTime = cT;
    firstTime = false;
    return;
  }
  if (cT - lastTime < rate) {
    return;
  }



  // Code de SORTIE
  // Code pour terminer l'état

  firstTime = true;
  etat = PIVOTE;
}
void pivote(unsigned long cT) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 1700;
  static bool firstTime = true;
  

  if (firstTime) {

    
    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, croisiere);

    // L'autre dans le sens opposé
    digitalWrite(m2_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    analogWrite(m2_pwm, croisiere);

    lastTime = cT;
    firstTime = false;
    return;
  }

  if (cT - lastTime < rate) {
    return;
  }


  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);

  firstTime = true;
  etat = MARCHE;
}
void afficherDistance(unsigned long cT){
  static unsigned long lastTime = 0;
  const unsigned long rate = 250;
 if (cT - lastTime < rate) { 
  Serial.print("Distance: ");
  Serial.println(distance);
    return;
  }
 
}




// Modèle de tâche avec retour de valeur.
// Exemple d'utilisation :
//   - Mesure de distance, température, etc.


#pragma endregion

void stateManager(unsigned long cT) {
  switch (etat) {

    case BOOT:
      boot(cT);
      break;

    case MARCHE:
      marche(cT);
      break;

    case LENT:
      lent(cT);
      break;

    case ARRET:
      arret(cT);
      break;

    case RECULE:
      recule(cT);
      break;

    case PIVOTE:
      pivote(cT);
      break;
  }
}

void boot(unsigned long cT) {

  distance = sensorDistance.distanceCm();
  etat = MARCHE;
}
#pragma region setup - loop
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(m1_pwm, OUTPUT);  //We have to set PWM pin as output
  pinMode(m1_in2, OUTPUT);  //Logic pins are also set as output
  pinMode(m1_in1, OUTPUT);

  pinMode(m2_pwm, OUTPUT);  //We have to set PWM pin as output
  pinMode(m2_in2, OUTPUT);  //Logic pins are also set as output
  pinMode(m2_in1, OUTPUT);
  led.setpin(LEDPIN);
}

void loop() {
  currentTime = millis();
  afficherDistance(currentTime);
  mesurerDistance(currentTime);
  
  stateManager(currentTime);
}
#pragma endregion