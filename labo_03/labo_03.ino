#include <MeAuriga.h>


MeUltrasonicSensor sonar(PORT_8);


#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0
#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
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
const int DEFAULT_SPEED = 100;
// PWM minimal pour que les roues tournent encore près de la cible
const int MIN_PWM = 35;

// Tolérances pour considérer que la cible est atteinte
const float DIST_TOLERANCE_CM = 0.5;
const float ANGLE_TOLERANCE_DEG = 1.0;

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

unsigned long currentTime = 0;

// Angle Z cumulatif (ne revient pas à -180 après 180)
double zAngle = 0.0;

bool debugMode = false;

// Prototypes des fonctions ayant des paramètres par défaut
// (l'IDE Arduino ne les génère pas automatiquement)
bool avanceTask(unsigned long ct, float distanceCm, int speed, bool init = false);
;
void stopState(unsigned long ct, bool newCmd = false);

void pivotState(unsigned long ct, bool newCmd = false, float angle = 0.0, int speed = DEFAULT_SPEED);
String msg = "";



AppState appState = ATTENTE;



// Les pragma region permettent de créer des zones de code fermable
#pragma region Modèles




// Modèle de tâche qui ne retourne aucune valeur.
void gyroTask(unsigned long ct) {
  static double previousZ = 0.0;
  static bool firstTime = true;

  gyro.update();
  double z = gyro.getAngleZ();

  if (firstTime) {
    firstTime = false;
    previousZ = z;
    return;
  }

  // getAngleZ() retourne un angle entre -180 et 180
  // On accumule la variation pour pouvoir pivoter de plus de 180 degrés
  double delta = z - previousZ;

  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;

  zAngle += delta;
  previousZ = z;
}

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);

  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);

  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);

  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);

  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

void stopMotors() {
  // setTarPWM pour que loop() ne relance pas les moteurs
  // setMotorPwm pour arrêter immédiatement
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

// Distance moyenne parcourue par les deux roues en cm
// Positif : vers l'avant
float getDistanceCm() {
  // La roue droite tourne dans le sens inverse pour avancer
  float avgDeg = (encoderLeft.getCurPos() - encoderRight.getCurPos()) / 2.0;

  return avgDeg / 360.0 * CIRC_WHEEL / 10.0;
}

// Garde la vitesse entre MIN_PWM et maxSpeed en conservant le signe
int limitSpeed(float value, int maxSpeed) {
  value = constrain(value, -maxSpeed, maxSpeed);

  if (fabs(value) < MIN_PWM) {
    value = value < 0 ? -MIN_PWM : MIN_PWM;
  }

  return (int)value;
}

// Avance de distanceCm centimètres (négatif : recule)
// - PD sur l'angle du gyro pour rester droit
// - PD sur la distance restante pour ralentir près de la cible
// Appeler avec init = true pour démarrer un nouveau déplacement
// Retourne true lorsque la distance est atteinte
bool avanceTask(unsigned long ct, float distanceCm, int speed, bool init) {
  static unsigned long previousTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static float startDistance = 0.0;

  static double previousAngleError = 0.0;
  static float previousDistError = 0.0;

  // PD de l'angle (pour rester droit)
  // higher kp = plus réactive, peu osciller
  // lower kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kpAngle = 15.0;
  const double kdAngle = 1.0;

  // PD de la distance (pour ralentir à l'approche)
  const float kpDist = 5.0;
  const float kdDist = 0.7;

  if (init) {
    previousTime = ct;
    zAngleGoal = zAngle;
    startDistance = getDistanceCm();
    previousAngleError = 0.0;
    previousDistError = distanceCm;

    return false;
  }

  if (ct - previousTime < rate) return false;

  previousTime = ct;

  float traveled = getDistanceCm() - startDistance;
  float distError = distanceCm - traveled;

  if (fabs(distError) <= DIST_TOLERANCE_CM) {
    stopMotors();
    return true;
  }

  // Vitesse de base selon la distance restante (signe = direction)
  float base = kpDist * distError + kdDist * (distError - previousDistError);
  previousDistError = distError;

  int baseSpeed = limitSpeed(base, speed);

  // Correction de l'angle
  double angleError = zAngle - zAngleGoal;
  double output = kpAngle * angleError + kdAngle * (angleError - previousAngleError);
  previousAngleError = angleError;

  msg = "dist : ";
  msg += traveled;
  msg += "\terr : ";
  msg += distError;
  msg += "\tz : ";
  msg += zAngle;
  msg += "\toutput : ";
  msg += output;

  // La correction s'applique dans le même sens sur les deux roues
  // donc elle fonctionne autant en avançant qu'en reculant
  encoderLeft.setTarPWM(baseSpeed - output);
  encoderRight.setTarPWM(-baseSpeed - output);

  return false;
}

// Pivote sur place de angleDeg degrés (négatif : autre sens)
// PD sur l'erreur d'angle du gyro
// Appeler avec init = true pour démarrer un nouveau pivot
// Retourne true lorsque l'angle est atteint
bool pivotTask(unsigned long ct, float angleDeg, int speed, bool init) {
  static unsigned long previousTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static double previousError = 0.0;

  const double kp = 3.0;
  const double kd = 4.0;

  if (init) {
    previousTime = ct;
    zAngleGoal = zAngle + angleDeg;
    previousError = angleDeg;

    return false;
  }

  if (ct - previousTime < rate) return false;

  previousTime = ct;

  double error = zAngleGoal - zAngle;

  if (fabs(error) <= ANGLE_TOLERANCE_DEG) {
    stopMotors();
    return true;
  }

  double output = kp * error + kd * (error - previousError);
  previousError = error;

  int pivotSpeed = limitSpeed(output, speed);

  msg = "z : ";
  msg += zAngle;
  msg += "\tgoal : ";
  msg += zAngleGoal;
  msg += "\terr : ";
  msg += error;
  msg += "\tpwm : ";
  msg += pivotSpeed;

  // Même signe sur les deux roues = rotation sur place
  encoderLeft.setTarPWM(pivotSpeed);
  encoderRight.setTarPWM(pivotSpeed);

  return false;
}
void stopState(unsigned long ct, bool newCmd) {
  static bool firstTime = true;

  if (newCmd) {
    firstTime = true;
    return;
  }

  if (firstTime) {
    firstTime = false;

    Serial.println(F("STOP"));
    msg = "";
  }

  stopMotors();
}


 void avanceState(unsigned long ct) {
  static bool firstTime = true;
  static float distanceDepart = 0;

  if (firstTime) {
    // Code d'ENTREE

    distanceDepart = getDistanceCm();

    // Eteindre les LED de progression
    for (int i = 1; i <= 7; i++) {
      led.setColorAt(i, 0, 0, 0);
    }

    // LED 8 verte : etat AVANCER_1
  

    // LED 11 verte : indicateur d'avancement
    

    led.show();

    firstTime = false;

    avanceTask(ct, 120, 100, true);
    return;
  }

  // Code d'EXECUTION

  bool done = avanceTask(ct, 120, 100);

  // Distance parcourue depuis l'entree dans l'etat
  float distance = abs(getDistanceCm() - distanceDepart);

  // Allumer progressivement les 7 LED sur 100 cm
  int nbLedAllume = constrain(
    (int)(distance * 7.0 / 100.0),
    0, 7
  );

  for (int i = 1; i <= 7; i++) {
    if (i <= nbLedAllume) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();

  // Code de TRANSITION
  if (done) {
    led.setColorAt(11, 0, 255, 0);
    led.show();
    firstTime = true;
    appState = PIVOTER_1;
  }
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

  
    led.show();

    lastTime = cT;
    firstTime = false;
    Serial.println("Attendre 3s");
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

void pivoter90Premier(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTREE

    // LED 8 jaune : PIVOTER_1
  
    led.show();
      for (int i = 1; i <= 7; i++) {
  led.setColorAt(i,0,0,0);
    }

    // Initialiser le pivot de 90 degres
    pivotTask(ct, 90, 100, true);

    firstTime = false;
    return;
  }

  // Code d'EXECUTION
  bool done = pivotTask(ct, 90, 100,false);

  // Code de TRANSITION
  if (done) {
    firstTime = true;
    appState = AVANCER_SONAR;
  }
}

void avancerDistance(unsigned long cT) {
  
  
}




float distanceSonar = 400;

void mesurerDistance(unsigned long cT) {
  
}
  void avancerSonar(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;

  static float distanceInitiale = 0;

  
  static float distanceCible = 0;
  static float distanceDepart = 0;
  static bool erreurSonar = false;

  if (firstTime) {
    // Code d'ENTREE

    // LED 8 bleue : AVANCER_SONAR
   

   //allumer LED 10
   
   

    // Reinitialiser les 7 LED de progression
    for (int i = 1; i <= 7; i++) {
      led.setColorAt(i, 0, 0, 0);
    }

    led.show();

    distanceInitiale = sonar.distanceCm();
Serial.println(distanceInitiale);
    distanceCible = distanceInitiale - 30.0;

    // Distance de depart selon les encodeurs
    distanceDepart = getDistanceCm();

    lastTime = ct;
    erreurSonar = false;

    // Ne pas avancer si la lecture est invalide
    if (distanceInitiale <= 0 || distanceInitiale > 300) {
      stopMotors();

      led.setColorAt(9, 255, 0, 0);
      led.show();

      erreurSonar = true;
      firstTime = false;
      return;
    }

    // L'obstacle est deja suffisamment proche
    if (distanceInitiale <= 32) {
      stopMotors();

      firstTime = true;
      appState = PIVOTER_2;
      return;
    }

    avanceTask(ct, distanceCible, 100, true);

    firstTime = false;
    return;
  }

  if (erreurSonar) return;

  // Code d'EXECUTION
  if (ct - lastTime < 20) return;
  lastTime = ct;

  mesurerDistance(ct);

  float distanceCm = distanceSonar;

  if (distanceCm > 0 && distanceCm <= 32) {
    stopMotors();

    firstTime = true;
    appState = PIVOTER_2;
    return;
  }

  bool done = avanceTask(ct, distanceCible, 100, false);

  float distance = abs(getDistanceCm() - distanceDepart);

  int nbLedAllume = constrain((int)(distance * 7.0 / distanceCible),0, 7);

  for (int i = 1; i <= 7; i++) {
    if (i <= nbLedAllume) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();

  if (done) {
    stopMotors();
 led.setColorAt(10, 0, 255, 0);
 led.show();
    firstTime = true;
    appState = PIVOTER_2;
  }

}


void pivoter90Second(unsigned long cT) {
   static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTREE
 for (int i = 1; i <= 7; i++) {
  led.setColorAt(i,0,0,0);
    }
   
  
    led.show();

    // Initialiser le pivot de 90 degres
    pivotTask(cT, -90, 100, true);

    firstTime = false;
    return;
  }

  // Code d'EXECUTION
  bool done = pivotTask(cT, -90, 100,false);

  // Code de TRANSITION
  if (done) {
    firstTime = true;
    appState = AVANCER_2;
  }
}

void avancer2(unsigned long cT) {
 static bool firstTime = true;
  static float distanceDepart = 0;

  if (firstTime) {
    // Code d'ENTREE

    distanceDepart = getDistanceCm();

    // Eteindre les LED de progression
    for (int i = 0; i <= 7; i++) {
      led.setColorAt(i, 0, 0, 0);
    }

   

  
    led.setColorAt(9, 0, 255, 0);

    led.show();

    firstTime = false;

    avanceTask(cT, 120, 100, true);
    return;
  }

  // Code d'EXECUTION

  bool done = avanceTask(cT, 120, 100);

  float distance = abs(getDistanceCm() - distanceDepart);

  int nbLedAllume = constrain((int)(distance * 7.0 / 100.0),0, 7);

  for (int i = 1; i <= 7; i++) {
    if (i <= nbLedAllume) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();

  // Code de TRANSITION
  if (done) {
     led.setColorAt(9, 0, 255, 0);
       led.show();
    firstTime = true;
    appState = PIVOTER_180;
  }
}

void pivoter180(unsigned long cT) {
   static bool firstTime = true;

  if (firstTime) {
    // Code d'ENTREE
 for (int i = 0; i <= 7; i++) {
  led.setColorAt(i,0,0,0);
    }
   
  led.show();

    // Initialiser le pivot de 90 degres
    pivotTask(cT, 180, 100, true);

    firstTime = false;
    return;
  }

  // Code d'EXECUTION
  bool done = pivotTask(cT, 180, 100,false);

  // Code de TRANSITION
  if (done) {
    firstTime = true;
    appState = TERMINE;
  }
}


void signalerFin(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 50;

  static bool firstTime = true;
  static bool allume = false;

  if (firstTime) {
    // Code d'ENTREE

    stopMotors();

    // Eteindre toutes les LED
    for (int i = 0; i <= 12; i++) {
      led.setColorAt(i, 0, 0, 0);
    }
    led.show();

    allume = false;
    lastTime = cT;
    firstTime = false;
    return;
  }

  // Temporisation
  if (cT - lastTime < rate) return;

  lastTime = cT;

  // Code d'EXECUTION
  allume = !allume;

  for (int i = 8; i <= 12; i++) {
    if (allume) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();
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
      avanceState(ct);
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

    case AVANCER_2:
      avancer2(ct);
      break;

    case PIVOTER_180:
      pivoter180(ct);
      break;

    case TERMINE:
      signalerFin(ct);
      break;
  }
}


#pragma region setup - loop
void setup() {
  // put your setup code here, to run once:
 
  
  Serial.begin(115200);
  
  encoderConfig();
  gyro.begin();
  led.setpin(LEDPIN);
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

 
}

void loop() {
   currentTime = millis();

  stateManager(currentTime);

  gyroTask(currentTime);
  encodersTask(currentTime);
  
}
#pragma endregion