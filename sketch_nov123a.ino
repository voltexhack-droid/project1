// ============================================================================
// Self-Balancing Robot — Arduino Uno + GY-61 (ADXL335) + L298N (Gentle Mode)
// ============================================================================

// --- GY-61 Analog Pins ---
const int X_PIN = A0;
const int Y_PIN = A1;
const int Z_PIN = A2;

// Set to true if tilting forward/back changes Y instead of X
const bool USE_Y_AXIS = false;

// Zero-g ADC midpoint (~1.65V on 5V Arduino ADC = ~338)
const float ZERO_G_ADC = 338.0;

// --- L298N Motor Driver Pins (Remove ENA & ENB jumpers on L298N!) ---
const int ENA = 5;   // Left motor speed (PWM)
const int IN1 = 7;   // Left motor direction 1
const int IN2 = 8;   // Left motor direction 2

const int IN3 = 9;   // Right motor direction 1
const int IN4 = 10;  // Right motor direction 2
const int ENB = 6;   // Right motor speed (PWM)

// --- Gentle Tuning Parameters ---
float Kp = 8.0;           // Gentle tilt response (raise slowly to 12-18 if too weak)
float Ki = 0.0;           // Off while tuning (prevents windup oscillation)
float Kd = 0.15;          // Light damping (prevents aggressive ADC spikes)

float targetAngle = 0.0;  // Set this to whatever Angle prints when held upright
const int MIN_PWM = 35;   // Soft starting PWM (raise to 45-55 only if wheels stall)
const int MAX_PWM = 115;  // Low speed cap (~45% power)
const float DEADBAND = 1.5;    // Degrees around upright where motors rest
const float FALL_ANGLE = 40.0; // Safety cutoff angle

// --- Motor Trim (0.50 to 1.00) ---
float leftScale = 1.0;
float rightScale = 1.0;

// --- Timing & Filter ---
const unsigned long LOOP_TIME_US = 10000; // 10 ms = 100 Hz
const float FILTER_ALPHA = 0.85;          // Stronger smoothing for analog sensor

float filteredAngle = 0.0;
float filteredRate = 0.0;
float prevAngle = 0.0;
float integral = 0.0;
unsigned long lastLoopMicros = 0;
int printCounter = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  filteredAngle = readTiltAngle();
  prevAngle = filteredAngle;
  lastLoopMicros = micros();

  Serial.println("GY-61 Gentle Mode Ready!");
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoopMicros < LOOP_TIME_US) {
    return;
  }

  float dt = (now - lastLoopMicros) / 1000000.0;
  lastLoopMicros = now;

  // 1. Read & strongly smooth tilt angle
  float rawAngle = readTiltAngle();
  filteredAngle = (FILTER_ALPHA * filteredAngle) + ((1.0 - FILTER_ALPHA) * rawAngle);

  // Smooth the rate of change so ADC noise doesn't cause violent spikes
  float rawRate = (filteredAngle - prevAngle) / dt;
  filteredRate = (0.80 * filteredRate) + (0.20 * rawRate);
  prevAngle = filteredAngle;

  // 2. Safety cutoff if robot falls over
  if (abs(filteredAngle - targetAngle) > FALL_ANGLE) {
    stopMotors();
    integral = 0.0;
    printDebug(filteredAngle, 0);
    return;
  }

  // 3. Deadband near upright to stop jittering
  float error = filteredAngle - targetAngle;
  if (abs(error) < DEADBAND && abs(filteredRate) < 10.0) {
    stopMotors();
    printDebug(filteredAngle, 0);
    return;
  }

  // 4. Gentle PID calculation
  integral += error * dt;
  float maxIntegral = (Ki > 0.0) ? ((float)MAX_PWM / Ki) : 0.0;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  float output = (Kp * error) + (Ki * integral) + (Kd * filteredRate);

  // 5. Drive motors
  int appliedPwm = driveMotors(output);

  printDebug(filteredAngle, (output >= 0) ? appliedPwm : -appliedPwm);
}

int readAveragedADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / samples);
}

float readTiltAngle() {
  float x = readAveragedADC(X_PIN, 12) - ZERO_G_ADC;
  float y = readAveragedADC(Y_PIN, 12) - ZERO_G_ADC;
  float z = readAveragedADC(Z_PIN, 12) - ZERO_G_ADC;

  float tiltAxis = USE_Y_AXIS ? y : x;
  return atan2(tiltAxis, z) * 180.0 / PI;
}

int driveMotors(float speed) {
  if (abs(speed) < 1.0) {
    stopMotors();
    return 0;
  }

  int pwm = MIN_PWM + (int)abs(speed);
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);

  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  int pwmLeft = constrain((int)(pwm * leftScale), 0, 255);
  int pwmRight = constrain((int)(pwm * rightScale), 0, 255);

  analogWrite(ENA, pwmLeft);
  analogWrite(ENB, pwmRight);

  return pwm;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void printDebug(float currentAngle, int currentPwm) {
  printCounter++;
  if (printCounter >= 5) {
    printCounter = 0;
    Serial.print("Angle: ");
    Serial.print(currentAngle, 1);
    Serial.print(" | PWM: ");
    Serial.println(currentPwm);
  }
}