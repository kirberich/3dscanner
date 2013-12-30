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
int[] stepping = {
  B000,
  B100,
  B010, 
  B110, 
  B111
}

int motorStep = 0;
int maxStep = 200;
int minimumStepDelay = 2;

String motorState = String("off");

void makeStep() {
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
  digitalWrite(switchPin, HIGH); 
  
  Serial.begin(9600);
} 

void loop()  {   
  if(Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'd':
        Serial.println(String("Current step: ") + motorStep);
        break;
      case 's':
        makeStep();
        Serial.println(motorStep);
        break;
      case 'r':
        resetMotor();
        Serial.println(String("Motor reset. (Step is ") + motorStep + String(")"));
    }
  }
}


