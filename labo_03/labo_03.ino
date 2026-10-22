#include <MeAuriga.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeGyro gyro(0, 0x69);
MeUltrasonicSensor sonar(PORT_7);


#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0

MeRGBLed led(PORT0, LEDNUM);

enum AppState {
  ATTENTE,
  AVANCER_1,
  PIVOTER_1,
  AVANCER_SONAR,
  PIVOTER_2,
  AVANCER_2,
  PIVOTER_180,
  TERMINE
};



AppState appState = ATTENTE;

const float distanceX = 100.0;
unsigned long currentTime = 0;

// Les pragma region permettent de créer des zones de code fermable
#pragma region Modèles
void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
    ;
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

float distanceParcourue(long depart1, long depart2) {
  float angle1 = abs(Encoder_1.getCurPos() - depart1);
  float angle2 = abs(Encoder_2.getCurPos() - depart2);

  float angleMoyen = (angle1 + angle2) / 2.0;
  float tour = angleMoyen / 360.0;
  float circonference = PI * 6.5;
  return tour * circonference;  //calcul inverse de goto();
}

void goStraight(short speed = 100, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  // Boucle de contrôle PD
  // Modifier les valeurs pour ajuster la réaction du robot
  // kp = coefficient proportionnel
  // kp plus élevé = plus réactif, peut avoir de l'oscillation
  // kp plus bas = moins réactif, mais moins d'oscillation
  //
  // kd = coefficient dérivé
  // kd plus élevé = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 15.0;
  const double kd = 1.0;

  // Premier appel de la fonction
  // On initialise les variables
  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    firstRun = 0;
    Serial.println("Setting speed");

    Encoder_2.setTarPWM(speed);
    Encoder_1.setTarPWM(-speed);

    return;
  }

  // On calcule l'erreur
  error = gyro.getAngleZ() - zAngleGoal;

  // On calcule la sortie
  output = kp * error + kd * (error - previousError);

  // On garde en mémoire l'erreur précédente
  previousError = error;

  // On applique la correction
   Encoder_2.setMotorPwm(speed - output);
    Encoder_1.setMotorPwm(-speed - output);
}
// Modèle de tâche qui ne retourne aucune valeur.
void xTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  unsigned long rate = 500;

  if (ct - lastTime < rate) {
    return;
  }

  lastTime = ct;

  // Faire le code de la tâche ici
}

void attendreDepart(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 3000;

  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.
    lastTime = cT;
    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = true;

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

    firstTime = true;

    appState = AVANCER_1;
  }
}


void avancerDistance(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 20;
  static long depart1 = 0;
  static long depart2 = 0;
  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.
    depart1 = Encoder_1.getCurPos();
    depart2 = Encoder_2.getCurPos();

    goStraight(100, 1);
    led.setColorAt(11, 255, 0, 0);
      lastTime = cT;
    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire
  float distance = distanceParcourue(depart1, depart2);
  int nbLed = 7;
  int nbLedAllume = map((long)distance, 0, (long)distanceX, 0, nbLed);
  for (int i = 0; i < nbLed; i++) {
    if (i < nbLedAllume) {
      led.setColorAt(i, 0, 0, 255);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }
  led.show();
  goStraight(100);
  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = (distance >= distanceX);

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
firstTime=true;
appState=PIVOTER_1;
  // TEST UNIQUEMENT

  }
}


void pivoter90Premier(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 20;

  static bool firstTime = true;
static float angleDepart=0;
  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.
    angleDepart=gyro.getAngleZ();
    
 Encoder_1.setTarPWM(130);
    Encoder_2.setTarPWM(130);
    firstTime = false;
    lastTime=cT;
    return;
  }


  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

float angleActuel=gyro.getAngleZ();
float angleParcouru=abs(angleActuel-angleDepart);

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = (angleParcouru>=90.0);

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

  
 Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0); 
     firstTime = true;
     appState = AVANCER_SONAR;
  }
}


void avancerSonar(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 20;
static float distanceInitiale = 0;
static long depart1 = 0;
static long depart2 = 0;
  static bool firstTime = true;


  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.
    distanceInitiale = sonar.distanceCm();
depart1 = Encoder_1.getCurPos();
    depart2 = Encoder_2.getCurPos();

    goStraight(100, 1);
     led.setColorAt(11, 0, 0, 0);
    led.setColorAt(10, 0, 255, 0);
      lastTime = cT;
    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire
float distanceCm = sonar.distanceCm();
float distance = distanceParcourue(depart1, depart2);

int nbLed = 7;
float distancePrevue = distanceInitiale - 30.0;

int nbLedAllume = 0;

if (distancePrevue > 0) {
  nbLedAllume = constrain(
    map((long)distance, 0, (long)distancePrevue, 0, nbLed),
    0, nbLed
  );
}
 for (int i = 0; i < nbLed; i++) {
    if (i < nbLedAllume) {
      led.setColorAt(i, 0, 0, 255);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }
  led.show();
  goStraight(100);
  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = (distanceCm>0 && distanceCm<=30);

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état
 Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0); 
    firstTime = true;

     appState = PIVOTER_2;
  }
}


void pivoter90Second(unsigned long cT) {
static unsigned long lastTime = 0;
  const int rate = 20;

  static bool firstTime = true;
static float angleDepart=0;
  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.
    angleDepart=gyro.getAngleZ();
    
 Encoder_1.setTarPWM(-130);
    Encoder_2.setTarPWM(-130);
    firstTime = false;
    lastTime=cT;
    return;
  }


  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

float angleActuel=gyro.getAngleZ();
float angleParcouru=abs(angleActuel-angleDepart);

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = (angleParcouru>=90.0);

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

  
 Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0); 
     firstTime = true;
     appState = AVANCER_2;
  }
}

void avancer2(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 500;

  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.

    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = true;

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

    firstTime = true;

    // appState = PROCHAIN_ETAT;
  }
}

void pivoter180(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 500;

  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.

    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = true;

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

    firstTime = true;

    // appState = PROCHAIN_ETAT;
  }
}


void signalerFin(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 500;

  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTRÉE c'est le
    //   code d'initialisation de l'état
    // Reset tes trucs
    // Exemples :
    //   Angle de référence, initialiser le lastTime,
    //   Chronomètre, etc.

    firstTime = false;
    return;
  }

  // Ligne nécessaire si l'on doit temporiser
  // les appels de cet état
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXÉCUTION de l'état
  // Code de la job à faire

  // Code de TRANSITION
  // Il s'agit de la transition qui permet de sortir de l'état
  // Qu'est-ce qui fait que l'on sort de l'état?
  bool transition = true;

  // Il est possible d'avoir plusieurs transitions

  if (transition) {
    // Code de SORTIE
    // Code pour terminer l'état

    firstTime = true;

    // appState = PROCHAIN_ETAT;
  }
}

// Modèle de tâche avec retour de valeur.
// Exemple d'utilisation :
//   - Mesure de distance, température, etc.
int xTaskWithReturn(unsigned long ct) {
  static unsigned long lastTime = 0;
  unsigned long rate = 500;
  static int lastResult = 0;
  int result = 0;

  if (ct - lastTime < rate) {
    return result;
  }

  lastTime = ct;

  // Faire le code de la tâche ici
  result = 1;

  lastResult = result;

  return result;
}

#pragma endregion

void stateManager(unsigned long ct) {

  switch (appState) {
    case ATTENTE:
      attendreDepart(ct);
      break;

    case AVANCER_1:
      avancerDistance(ct);
      break;

    case PIVOTER_1:
      pivoter90Premier(ct);
      break;
 case AVANCER_SONAR:
      avancerSonar(ct);
      break;
 case PIVOTER_2:
      pivoter90Second(ct);
      break;
  }
}


#pragma region setup - loop
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  gyro.begin();
   led.setpin(LEDPIN); 
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
}

void loop() {
  currentTime = millis();
  gyro.update();


 
    Encoder_1.loop();
    Encoder_2.loop();
  stateManager(currentTime);

}
#pragma endregion