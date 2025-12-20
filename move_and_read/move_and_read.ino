int ENABLE_PIN = 8;
int M1_STEP_PIN = 2;
int M1_DIR_PIN = 5;
int M2_STEP_PIN = 3;
int M2_DIR_PIN = 6;

// Distance Sensor Pin
int analogPin = A2;

// Variables
int sensorValue = 0;
float voltage = 0.0;

const int MICROSTEPS = 16;
const unsigned int STEP_PULSE_US = 3125;

const float PITCH_MAX_DEG = 20.0;
const long M2_MICROSTEPS_PER_REV = 200L * MICROSTEPS;

long m1MicrostepPos = 0;
long m2MicrostepPos = 0;
unsigned long sampleIndex = 0;

int m2Dir = 1;

long pitchMaxMicrosteps = 0;
bool scanComplete = false;

void stepPulses(int stepPin, int pulses);
void emitEvent(const char* type);
void printSample(int phase, int dir);
int readMedianAnalog5(int pin);

void setup() {
  // Initialize serial communication for sensor readings
  Serial.begin(9600);
  // Configure stepper motor pins as outputs
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(analogPin, INPUT);
  
  // Enable the stepper drivers (LOW typically enables)
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(M2_DIR_PIN, LOW);
  pitchMaxMicrosteps = (long)((PITCH_MAX_DEG / 360.0) * (float)M2_MICROSTEPS_PER_REV + 0.5);
  Serial.println("type,ms,sample,phase,m1_dir,m2_dir,m1_us,m2_us,adc,voltage");
  emitEvent("START");
}

void loop() {
  if (scanComplete) {
    delay(1000);
    return;
  }

  // Set direction forward
  digitalWrite(M1_DIR_PIN, LOW);

  // Rotate forward 200 steps (one full rotation for most steppers)
  for (int i = 0; i < 100; i++) {
    // Step both motors
    stepPulses(M1_STEP_PIN, MICROSTEPS);
    m1MicrostepPos += MICROSTEPS;
    printSample(i, 1);
  }

  delay(100);

  digitalWrite(M2_DIR_PIN, (m2Dir > 0) ? LOW : HIGH);
  stepPulses(M2_STEP_PIN, MICROSTEPS);
  m2MicrostepPos += (long)m2Dir * MICROSTEPS;

  long m2Abs = (m2MicrostepPos >= 0) ? m2MicrostepPos : -m2MicrostepPos;
  bool stopAfterBackward = (m2Abs >= pitchMaxMicrosteps);

  // Set direction reverse
  digitalWrite(M1_DIR_PIN, HIGH);
  
  // Rotate backward 200 steps
  for (int i = 0; i < 100; i++) {
    // Step both motors
    stepPulses(M1_STEP_PIN, MICROSTEPS);
    m1MicrostepPos -= MICROSTEPS;
    printSample(i, -1);
  }
  
  delay(100);

  if (stopAfterBackward) {
    emitEvent("DONE");
    digitalWrite(ENABLE_PIN, HIGH);
    scanComplete = true;
    return;
  }

  digitalWrite(M2_DIR_PIN, (m2Dir > 0) ? LOW : HIGH);
  stepPulses(M2_STEP_PIN, MICROSTEPS);
  m2MicrostepPos += (long)m2Dir * MICROSTEPS;

}

void stepPulses(int stepPin, int pulses) {
  for (int i = 0; i < pulses; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
}

void emitEvent(const char* type) {
  Serial.print(type);
  Serial.print(",");
  Serial.print(millis());
  Serial.println(",,,,,,,,");
}

void printSample(int phase, int dir) {
  sensorValue = readMedianAnalog5(analogPin);
  voltage = (sensorValue * 5.0) / 1023.0;

  Serial.print("DATA");
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(sampleIndex++);
  Serial.print(",");
  Serial.print(phase);
  Serial.print(",");
  Serial.print(dir);
  Serial.print(",");
  Serial.print(m2Dir);
  Serial.print(",");
  Serial.print(m1MicrostepPos);
  Serial.print(",");
  Serial.print(m2MicrostepPos);
  Serial.print(",");
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.println(voltage, 4);
}

int readMedianAnalog5(int pin) {
  int v[5];
  v[0] = analogRead(pin);
  delay(2);
  v[1] = analogRead(pin);
  delay(2);
  v[2] = analogRead(pin);
  delay(2);
  v[3] = analogRead(pin);
  delay(2);
  v[4] = analogRead(pin);

  for (int i = 1; i < 5; i++) {
    int key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }

  return v[2];
}
