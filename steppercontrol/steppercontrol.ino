int laserPins[] = {2, 3};
int numLasers = 2;
int enablePin = 4;
int MS1Pin = 5;
int MS2Pin = 6;
int MS3Pin = 7;
int resetPin = 9;
int sleepPin = 10;
int motorPin = 11;
int directionPin = 12;
int switchPin = 13;

// Stepping: Full, half, quarter, eigth, sixteenth
int stepping[] = {
  B000,
  B100,
  B010, 
  B110, 
  B111
};

int motorStep = 0;
int maxStep = 200;
int minimumStepDelay = 2;
int laserState = false;
boolean motorState = true;

unsigned long lastStep = 0;

void setMotor(boolean new_state) {
  digitalWrite(enablePin, !new_state);
  motorState = new_state;
}

void disableIdleMotor() {
  if (millis() - lastStep > 10000) {
    setMotor(false);
    lastStep = millis();
  }
}

void makeStep() {
  if (!motorState) {
    setMotor(true);
  }
  lastStep = millis();
  digitalWrite(motorPin, HIGH);
  digitalWrite(motorPin, LOW);
  motorStep += 1;
  if (motorStep > maxStep) {
    motorStep = 0;
  } 
}

void resetMotor() {
  for (int i = motorStep; i < maxStep; i++) {
    makeStep();
    delay(minimumStepDelay);
  }
}

void setup()  { 
  pinMode(switchPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  setMotor(false);
  
  for(int i=0; i<numLasers; i++) {
    pinMode(laserPins[i], OUTPUT);
  }
  digitalWrite(switchPin, HIGH); 
  
  Serial.begin(9600);
} 

void loop()  {  
  disableIdleMotor();
  
  if(Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'd':
        Serial.println(String("Current step: ") + motorStep);
        Serial.println(String("Laser status: ") + laserState);
        break;
      case 'M':
        setMotor(true);
        Serial.println("Motor enabled");
        break;
      case 'm':
        setMotor(false);
        Serial.println("Motor disabled");
        break;
      case 's':
        makeStep();
        Serial.println(motorStep);
        break;
      case 'r':
        resetMotor();
        Serial.println(String("Motor reset. (Step is ") + motorStep + String(")"));
        break;
      case 'l':
        laserState = false;
        for(int i=0; i<numLasers; i++) {
          digitalWrite(laserPins[i], laserState);
        }
        break;
      case 'L':
        laserState = true;
        for(int i=0; i<numLasers; i++) {
          digitalWrite(laserPins[i], laserState);
        }
        break;
    }
  }
}


