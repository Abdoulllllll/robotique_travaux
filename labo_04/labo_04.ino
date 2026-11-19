#include <MeAuriga.h>
#include <Adafruit_seesaw.h>

#define NB_IR 5
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

Adafruit_seesaw ss;
MeUltrasonicSensor sonar(PORT_8);
MeRGBLed led(PORT0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

enum AppState {
  BOOT,
  ATTENTE,
  LAB3,
  CALIB,
  SUIVRE_LIGNE,
  CMD_INVALIDE,
  NON_CALIB
};

enum EtapeLab3 {
  L3_AVANCER_1,
  L3_PIVOTER_1,
  L3_AVANCER_SONAR,
  L3_PIVOTER_2,
  L3_AVANCER_2,
  L3_PIVOTER_180,
  L3_TERMINE
};

enum EtapeSuivi {
  LIGNE_SUIVI,
  LIGNE_PIVOT_180
};

AppState appState = BOOT;
EtapeLab3 etapeLab3 = L3_AVANCER_1;
EtapeSuivi etapeSuivi = LIGNE_SUIVI;

bool firstTimeLab3 = true;
bool firstTimeEtape = true;
bool firstTimeCalib = true;
bool firstTimeSuivi = true;
bool firstTimeAttente = true;
bool firstTimeCmdInvalide = true;
bool firstTimeNonCalib = true;

bool isCalibrated = false;

unsigned long currentTime = 0;

const int DEFAULT_SPEED = 100;
const int MIN_PWM = 35;
const float DIST_TOLERANCE_CM = 0.5;
const float ANGLE_TOLERANCE_DEG = 1.0;

int sensorValues[NB_IR];
int valMin[NB_IR] = {1023, 1023, 1023, 1023, 1023};
int valMax[NB_IR] = {0, 0, 0, 0, 0};
int seuil[NB_IR];

double zAngle = 0.0;

float pidIntegral = 0;
float pidLastError = 0;

String commandeRecue = "";
String msg = "";

void encoderConfig();
void gyroTask(unsigned long ct);
void encodersTask(unsigned long ct);
void stopMotors();
float getDistanceCm();
int limitSpeed(float value, int maxSpeed);
bool avanceTask(unsigned long ct, float distanceCm, int speed, bool init = false);
bool pivotTask(unsigned long ct, float angleDeg, int speed, bool init = false);
void calibrer();
void calculerSeuils();
void resetPID();
float computePID(float position, float consigne = 0.0f);
float lirePositionLigne();
void appliquerCorrection(float correction);
void lireBluetooth();
void traiterCommande(String commande);
void resetEtats();
void stateManager(unsigned long ct);
void bootState(unsigned long ct);
void attenteState(unsigned long ct);
void labo3State(unsigned long ct);
void calibState(unsigned long ct);
void suivreLigneState(unsigned long ct);
void cmdInvalideState(unsigned long ct);
void nonCalibState(unsigned long ct);

void rightEncoderInterrupt() {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt() {
  if (digitalRead(encoderLeft.getPortB()) == 0) {
    encoderLeft.pulsePosMinus();
  } else {
    encoderLeft.pulsePosPlus();
  }
}

void setup() {
  Serial.begin(115200);

  encoderConfig();
  gyro.begin();
  led.setpin(LEDPIN);

  led.setColor(0, 0, 0, 0);
  led.show();

  stopMotors();

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1) {
      stopMotors();
    }
  }

  Serial.println("Connexion reussie au LyneTracker");
  Serial.println("Etat : BOOT");
}

void loop() {
  currentTime = millis();

  gyroTask(currentTime);
  encodersTask(currentTime);
  lireBluetooth();
  stateManager(currentTime);
}

void stateManager(unsigned long ct) {
  switch (appState) {
    case BOOT:
      bootState(ct);
      break;

    case ATTENTE:
      attenteState(ct);
      break;

    case LAB3:
      labo3State(ct);
      break;

    case CALIB:
      calibState(ct);
      break;

    case SUIVRE_LIGNE:
      suivreLigneState(ct);
      break;

    case CMD_INVALIDE:
      cmdInvalideState(ct);
      break;

    case NON_CALIB:
      nonCalibState(ct);
      break;
  }
}

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

  double delta = z - previousZ;

  if (delta > 180) {
    delta -= 360;
  } else if (delta < -180) {
    delta += 360;
  }

  zAngle += delta;
  previousZ = z;
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);

  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);

  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);

  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

void stopMotors() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

float getDistanceCm() {
  float avgDeg = (encoderLeft.getCurPos() - encoderRight.getCurPos()) / 2.0;
  return avgDeg / 360.0 * CIRC_WHEEL / 10.0;
}

int limitSpeed(float value, int maxSpeed) {
  value = constrain(value, -maxSpeed, maxSpeed);

  if (fabs(value) < MIN_PWM) {
    value = value < 0 ? -MIN_PWM : MIN_PWM;
  }

  return (int)value;
}

bool avanceTask(unsigned long ct, float distanceCm, int speed, bool init) {
  static unsigned long previousTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static float startDistance = 0.0;
  static double previousAngleError = 0.0;
  static float previousDistError = 0.0;

  const double kpAngle = 15.0;
  const double kdAngle = 1.0;
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

  float base = kpDist * distError + kdDist * (distError - previousDistError);
  previousDistError = distError;

  int baseSpeed = limitSpeed(base, speed);

  double angleError = zAngle - zAngleGoal;
  double output = kpAngle * angleError + kdAngle * (angleError - previousAngleError);
  previousAngleError = angleError;

  encoderLeft.setTarPWM(baseSpeed - output);
  encoderRight.setTarPWM(-baseSpeed - output);

  return false;
}

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

  encoderLeft.setTarPWM(pivotSpeed);
  encoderRight.setTarPWM(pivotSpeed);

  return false;
}

void calibrer() {
  for (int i = 0; i < NB_IR; i++) {
    int valeur = ss.analogRead(i);

    if (valeur < valMin[i]) {
      valMin[i] = valeur;
    }

    if (valeur > valMax[i]) {
      valMax[i] = valeur;
    }
  }
}

void calculerSeuils() {
  for (int i = 0; i < NB_IR; i++) {
    seuil[i] = (valMin[i] + valMax[i]) / 2;
  }
}

void resetPID() {
  pidIntegral = 0;
  pidLastError = 0;
}

float computePID(float position, float consigne) {
  const float kp = 0.1;
  const float ki = 0.01;
  const float kd = 0.01;

  float error = position - consigne;

  pidIntegral += error;

  const float integralLimit = 1000;
  pidIntegral = constrain(pidIntegral, -integralLimit, integralLimit);

  float derivative = error - pidLastError;
  pidLastError = error;

  float output = kp * error + ki * pidIntegral + kd * derivative;

  return output;
}

float lirePositionLigne() {
  float numerateur = 0;
  float denominateur = 0;
  bool ligneDetectee = false;

  for (int i = 0; i < NB_IR; i++) {
    int valeur = ss.analogRead(i);

    if (valMax[i] > valMin[i]) {
      if (valeur < seuil[i]) {
        ligneDetectee = true;
      }

      float valNorm = (valeur - valMin[i]) * 1000.0 / (valMax[i] - valMin[i]);
      valNorm = constrain(valNorm, 0.0, 1000.0);
      valNorm = 1000.0 - valNorm;

      numerateur += valNorm * (i - 2);
      denominateur += valNorm;
    }
  }

  if (!ligneDetectee || denominateur == 0) {
    return 9999;
  }

  return 1000.0 * numerateur / denominateur;
}

void appliquerCorrection(float correction) {
  int vitesseGauche = constrain(100 + correction, 0, 180);
  int vitesseDroite = constrain(100 - correction, 0, 180);

  encoderLeft.setTarPWM(vitesseGauche);
  encoderRight.setTarPWM(-vitesseDroite);
}

void resetEtats() {
  firstTimeLab3 = true;
  firstTimeEtape = true;
  firstTimeCalib = true;
  firstTimeSuivi = true;
  firstTimeAttente = true;
  firstTimeCmdInvalide = true;
  firstTimeNonCalib = true;

  etapeLab3 = L3_AVANCER_1;
  etapeSuivi = LIGNE_SUIVI;

  resetPID();
}

void lireBluetooth() {
  while (Serial.available() > 0) {
    char caractere = (char)Serial.read();

    if (commandeRecue.length() == 0 && (uint8_t)caractere == 0xFF) {
      continue;
    }

    if (commandeRecue.length() == 0 && (uint8_t)caractere == 0x55) {
      continue;
    }

    if (caractere == '\n' || caractere == '\r') {
      commandeRecue.trim();

      if (commandeRecue.length() > 0) {
        traiterCommande(commandeRecue);
      }

      commandeRecue = "";
    } else {
      if (commandeRecue.length() < 32) {
        commandeRecue += caractere;
      } else {
        commandeRecue = "";
      }
    }
  }
}

void traiterCommande(String commande) {
  commande.trim();
  commande.toLowerCase();

  if (commande.length() == 0) return;

  Serial.print("cmd recu : ");
  Serial.println(commande);

  if (commande == "stop") {
    stopMotors();
    led.setColor(0, 0, 0, 0);
    led.show();

    if (appState == CALIB) {
      isCalibrated = false;
    }

    resetEtats();
    appState = ATTENTE;
    return;
  }

  if (appState == BOOT) return;

  if (appState != ATTENTE) return;

  stopMotors();
  led.setColor(0, 0, 0, 0);
  led.show();

  resetEtats();

  if (commande == "lab3") {
    appState = LAB3;
  } else if (commande == "cal") {
    appState = CALIB;
  } else if (commande == "ligne") {
    if (isCalibrated) {
      appState = SUIVRE_LIGNE;
    } else {
      appState = NON_CALIB;
    }
  } else {
    Serial.println("cmd INCONNUE");
    appState = CMD_INVALIDE;
  }
}

void bootState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;

  if (firstTime) {
    stopMotors();
    lastTime = ct;
    firstTime = false;
    return;
  }

  if (ct - lastTime < 3000) return;

  appState = ATTENTE;
  firstTimeAttente = true;
}
void afficherCapteurs(unsigned long ct) {
  static unsigned long lastTime = 0;

  if (ct - lastTime < 200) return;
  lastTime = ct;

  for (int i = 0; i < NB_IR; i++) {
    int valeur = ss.analogRead(i);

    Serial.print("IR");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(valeur);
    Serial.print("/");
    Serial.print(seuil[i]);
    Serial.print("  ");
  }

  Serial.println();
}
void attenteState(unsigned long ct) {
  if (firstTimeAttente) {
    stopMotors();
    Serial.println("Etat : ATTENTE");
    firstTimeAttente = false;
    return;
  }
  if (isCalibrated) {
  afficherCapteurs(ct);
}
}

void calibState(unsigned long ct) {
  if (firstTimeCalib) {
    Serial.println("Etat : CALIB");

    stopMotors();
    isCalibrated = false;

    for (int i = 0; i < NB_IR; i++) {
      valMin[i] = 1023;
      valMax[i] = 0;
    }

    pivotTask(ct, 360, 100, true);

    firstTimeCalib = false;
    return;
  }

  calibrer();

  bool termine = pivotTask(ct, 360, 100, false);

  if (termine) {
    stopMotors();

    bool calibrationValide = true;

    for (int i = 0; i < NB_IR; i++) {
      if (valMax[i] <= valMin[i]) {
        calibrationValide = false;
      }
    }

    if (calibrationValide) {
      calculerSeuils();
      isCalibrated = true;
      Serial.println("Calibration terminee !");
    } else {
      isCalibrated = false;
      Serial.println("Calibration invalide !");
    }

    firstTimeCalib = true;
    firstTimeAttente = true;
    appState = ATTENTE;
  }
}

void cmdInvalideState(unsigned long ct) {
  static unsigned long startTime = 0;
  static unsigned long lastTime = 0;
  static bool ledOn = false;

  if (firstTimeCmdInvalide) {
    stopMotors();

    startTime = ct;
    lastTime = ct;
    ledOn = false;

    Serial.println("Etat : CMD_INVALIDE");

    firstTimeCmdInvalide = false;
    return;
  }

  if (ct - lastTime >= 250) {
    lastTime = ct;
    ledOn = !ledOn;

    for (int i = 1; i <= LEDNUM; i++) {
      if (ledOn) {
        led.setColor(i, 255, 0, 0);
      } else {
        led.setColor(i, 0, 0, 0);
      }
    }

    led.show();
  }

  if (ct - startTime >= 3000) {
    led.setColor(0, 0, 0, 0);
    led.show();

    firstTimeCmdInvalide = true;
    firstTimeAttente = true;
    appState = ATTENTE;
  }
}

void nonCalibState(unsigned long ct) {
  static unsigned long startTime = 0;
  static unsigned long lastTime = 0;
  static bool ledOn = false;

  if (firstTimeNonCalib) {
    stopMotors();

    startTime = ct;
    lastTime = ct;
    ledOn = false;

    Serial.println("Etat : NON_CALIB");

    firstTimeNonCalib = false;
    return;
  }

  if (ct - lastTime >= 250) {
    lastTime = ct;
    ledOn = !ledOn;

    for (int i = 1; i <= LEDNUM; i++) {
      if (ledOn) {
        led.setColor(i, 255, 255, 0);
      } else {
        led.setColor(i, 0, 0, 0);
      }
    }

    led.show();
  }

  if (ct - startTime >= 3000) {
    led.setColor(0, 0, 0, 0);
    led.show();

    firstTimeNonCalib = true;
    firstTimeAttente = true;
    appState = ATTENTE;
  }
}

void labo3State(unsigned long ct) {
  if (firstTimeLab3) {
    Serial.println("Etat : LAB3");

    etapeLab3 = L3_AVANCER_1;
    firstTimeEtape = true;
    firstTimeLab3 = false;
    return;
  }

  switch (etapeLab3) {
    case L3_AVANCER_1:
      if (firstTimeEtape) {
        avanceTask(ct, 100, 100, true);
        firstTimeEtape = false;
      } else if (avanceTask(ct, 100, 100, false)) {
        etapeLab3 = L3_PIVOTER_1;
        firstTimeEtape = true;
      }
      break;

    case L3_PIVOTER_1:
      if (firstTimeEtape) {
        pivotTask(ct, 90, 100, true);
        firstTimeEtape = false;
      } else if (pivotTask(ct, 90, 100, false)) {
        etapeLab3 = L3_AVANCER_SONAR;
        firstTimeEtape = true;
      }
      break;

    case L3_AVANCER_SONAR: {
      float distance = sonar.distanceCm();

      if (distance > 0 && distance <= 32) {
        stopMotors();
        etapeLab3 = L3_PIVOTER_2;
        firstTimeEtape = true;
      } else if (distance > 32) {
        encoderLeft.setTarPWM(100);
        encoderRight.setTarPWM(-100);
      } else {
        stopMotors();
      }

      firstTimeEtape = false;
      break;
    }

    case L3_PIVOTER_2:
      if (firstTimeEtape) {
        pivotTask(ct, -90, 100, true);
        
        firstTimeEtape = false;
      } else if (pivotTask(ct, -90, 100, false)) {
        etapeLab3 = L3_AVANCER_2;
        firstTimeEtape = true;
      }
      break;

    case L3_AVANCER_2:
      if (firstTimeEtape) {
        avanceTask(ct, 100, 100, true);
        firstTimeEtape = false;
      } else if (avanceTask(ct, 100, 100, false)) {
        etapeLab3 = L3_PIVOTER_180;
        firstTimeEtape = true;
      }
      break;

    case L3_PIVOTER_180:
      if (firstTimeEtape) {
        pivotTask(ct, 180, 100, true);
        firstTimeEtape = false;
      } else if (pivotTask(ct, 180, 100, false)) {
        etapeLab3 = L3_TERMINE;
        firstTimeEtape = true;
      }
      break;

    case L3_TERMINE:
      stopMotors();
      Serial.println("LAB3 termine !");

      firstTimeLab3 = true;
      firstTimeEtape = true;
      firstTimeAttente = true;

      appState = ATTENTE;
      break;
  }
}

void suivreLigneState(unsigned long ct) {
  static unsigned long dernierPID = 0;
  static unsigned long derniereLigneVue = 0;

  if (firstTimeSuivi) {
    Serial.println("Etat : SUIVRE_LIGNE");

    stopMotors();
    resetPID();

    etapeSuivi = LIGNE_SUIVI;

    dernierPID = ct;
    derniereLigneVue = ct;

    firstTimeSuivi = false;
    return;
  }

  switch (etapeSuivi) {
    case LIGNE_SUIVI: {
      if (ct - dernierPID < 20) return;

      dernierPID = ct;

      float position = lirePositionLigne();

      if (position != 9999) {
        derniereLigneVue = ct;

        float correction = computePID(position, 0);
        appliquerCorrection(correction);
      } else {
        stopMotors();
        resetPID();

        if (ct - derniereLigneVue >= 250) {
          Serial.println("Fin de ligne : pivot 180");

          pivotTask(ct, 180, 100, true);
          etapeSuivi = LIGNE_PIVOT_180;
        }
      }

      break;
    }

    case LIGNE_PIVOT_180: {
      bool termine = pivotTask(ct, 180, 100, false);

      if (termine) {
        stopMotors();
        resetPID();

        dernierPID = ct;
        derniereLigneVue = ct;

        etapeSuivi = LIGNE_SUIVI;

        Serial.println("Reprise du suivi de ligne");
      }

      break;
    }
  }
}
