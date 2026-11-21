// ---------------------------------------------------------------- //
// Arduino Jeep Bot - Robot Cleaner
// Drives for 5s, stops, spins in place for 5s, and repeats until
// an object is found (up to 30 cm). Approaches it, and at 15 cm
// lowers the shovel to collect.
// The shovel stays up by default and only lowers during collection.
// ---------------------------------------------------------------- //

#include <Servo.h>

// ----------------------------- PINS ----------------------------- //
int echoPin  = 8;   // D8  -> HC-SR04 Echo (sensor facing FORWARD)
int trigPin  = 7;   // D7  -> HC-SR04 Trig
int in1      = 4;   // Motor A - Pin 1
int in2      = 5;   // Motor A - Pin 2
int in3      = 9;   // Motor B - Pin 1
int in4      = 10;  // Motor B - Pin 2
int servoPin = 6;   // D6  -> Shovel servo signal

// ------------------- SETTINGS (calibrate on robot) -------------- //
const int SHOVEL_UP   = 100;  // Shovel angle up (raised) - default position
const int SHOVEL_DOWN = 20;   // Shovel angle down (touching the floor)

const int DETECT_DIST = 30;   // cm: sees object up to here -> moves towards it
const int PICKUP_DIST = 15;   // cm: got close -> lowers shovel and collects

const unsigned long WANDER_TIME    = 5000; // ms driving straight before stopping and spinning

// ---- LOW POWER MODE (weak 9V battery): slower and smoother ----
// With low voltage, motors move less per millisecond, so times are longer.
const unsigned long SPIN_TIME = 5000; // ms (real time clock) spinning in place searching for object
const int SPIN_STEP_MS   = 200; // ms per spin "step" (larger pulse overcomes friction)
const int SPIN_SETTLE_MS = 150; // ms stopped after each step (voltage recovers + stable reading)

const int PUSH_MS = 800;        // ms moving forward with shovel down (0 = just lowering is enough)
const int BACK_MS = 600;        // ms reversing after lifting shovel with trash

const int STAGGER_MS     = 50;  // ms between turning on motor A and B (prevents simultaneous current spike)
const int SHOVEL_STEP_MS = 30;  // ms per servo degree (15 = fast, 30 = smooth and draws less peak current)

// ---------------------------- VARIABLES -------------------------- //
Servo shovel;
int shovelAngle = SHOVEL_UP;

long duration;
int distance;

enum State { WANDER, SCAN, APPROACH };
State state = WANDER;
unsigned long stateStart = 0;  // When the current state started (ms)
int scanDir = 0;               // 0 = left, 1 = right

// ------------------------------ SETUP ---------------------------- //
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  shovel.attach(servoPin);
  shovel.write(SHOVEL_UP);    // Shovel starts (and stays) up

  randomSeed(analogRead(A0)); // A0 floating = noise = different seed on every boot
  stateStart = millis();

  Serial.begin(9600);
}

// ------------------------------ LOOP ----------------------------- //
void loop() {
  distance = readDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= DETECT_DIST) {
    // 1. Spotted something up to 30 cm for the first time: stop and confirm reading
    if (state != APPROACH) {
      stopMotors();
      delay(SPIN_SETTLE_MS);
      if (readStableDistance() <= DETECT_DIST) {
        state = APPROACH;     // Confirmed: it is an object
      }
      // If not confirmed, it was a false alarm: proceed with normal search below
    }

    // 2. Approaching: drive until reaching 15 cm
    if (state == APPROACH) {
      if (distance <= PICKUP_DIST) {
        stopMotors();
        delay(SPIN_SETTLE_MS);
        if (readStableDistance() <= PICKUP_DIST) {
          pickUpTrash();      // Arrived: lower shovel and collect
          return;
        }
      }
      forward();
      delay(30);
      return;
    }
  } else if (state == APPROACH) {
    // Lost sight of the object during approach: resume spinning search
    stopMotors();
    state = SCAN;
    stateStart = millis();   // Start counting 5s spin time
    scanDir = random(2);
  }

  // 3. Nothing in sight: drive 5s, stop and spin 5s in place, repeat
  if (state == WANDER) {
    forward();
    if (millis() - stateStart >= WANDER_TIME) {
      stopMotors();
      delay(SPIN_SETTLE_MS);
      state = SCAN;
      stateStart = millis();  // Start counting 5s spin time
      scanDir = random(2);    // Randomize sweep direction
    }
    delay(30);
  } else {
    scanStep();
  }
}

// ------------------------ SENSOR READING ------------------------- //
int readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 25000 us timeout (~4m) to avoid the 804 cm bug
  duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return 999; // Nothing read = path completely clear
  }
  return duration * 0.034 / 2;
}

// Reads 3 times and returns the middle value (ignores glitchy readings)
int readStableDistance() {
  int a = readDistance();
  delay(30);
  int b = readDistance();
  delay(30);
  int c = readDistance();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

// ---------------------- SPIN IN PLACE ---------------------------- //
// A single spin step: turns on, stops, and waits to settle.
// (This way the sensor reads while the robot is stopped, making turns more predictable.)
void spinStep(int dir) {
  if (dir == 0) {
    turnLeft();
  } else {
    turnRight();
  }
  delay(SPIN_STEP_MS);
  stopMotors();
  delay(SPIN_SETTLE_MS);
}

// Spins in steps until completing "ms" of active time
void spinInPlace(unsigned long ms, int dir) {
  unsigned long spun = 0;
  while (spun < ms) {
    spinStep(dir);
    spun += SPIN_STEP_MS;
  }
}

// Sweep: one step per loop() iteration; loop() reads the sensor between steps
void scanStep() {
  spinStep(scanDir);

  // Spun for 5s without finding anything: resume driving straight for 5s
  if (millis() - stateStart >= SPIN_TIME) {
    state = WANDER;
    stateStart = millis();
  }
}

// ------------------------- TRASH COLLECTION ---------------------- //
void pickUpTrash() {
  stopMotors();
  delay(200);

  moveShovel(SHOVEL_DOWN);   // 1. Lower shovel only now (motors stopped)

  forward();                 // 2. Move forward slightly to push object into the shovel
  delay(PUSH_MS);
  stopMotors();
  delay(200);

  moveShovel(SHOVEL_UP);     // 3. Return shovel back up with the trash

  backward();                // 4. Reverse to back away
  delay(BACK_MS);
  stopMotors();
  delay(100);

  // 5. Turn a random angle (in place) and restart the search cycle
  spinInPlace(random(300, 900), random(2));
  state = WANDER;
  stateStart = millis();
}

// Moves the servo slowly (1 degree at a time) to prevent jerking and high current draw
void moveShovel(int target) {
  while (shovelAngle != target) {
    if (target > shovelAngle) {
      shovelAngle++;
    } else {
      shovelAngle--;
    }
    shovel.write(shovelAngle);
    delay(SHOVEL_STEP_MS);
  }
}

// ----------------------------- MOTORS --------------------------- //
// If any movement is reversed, swap HIGH/LOW (or motor wires) here.
// Staggered startup: turns on motor A, waits a bit, then turns on motor B,
// preventing both from pulling a current spike at the exact same time (weak battery friendly).
// The robot always STOPS before changing direction (no direct reversal = no extra peak current).

int lastA = 0;   // Last motor A command: 1 = forward, -1 = backward, 0 = stopped
int lastB = 0;   // Last motor B command

void motorA(int dir) {
  if (dir == 1)       { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else if (dir == -1) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
}

void motorB(int dir) {
  if (dir == 1)       { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  else if (dir == -1) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  }
  else                { digitalWrite(in3, LOW);  digitalWrite(in4, LOW);  }
}

void drive(int a, int b) {
  if (a == lastA && b == lastB) return;          // Already in this state: do nothing
  motorA(a);
  if (b != 0 && lastB == 0) delay(STAGGER_MS);   // B was stopped: wait before turning on
  motorB(b);
  lastA = a;
  lastB = b;
}

void forward()    { drive( 1,  1); }
void backward()   { drive(-1, -1); }
void turnLeft()   { drive(-1,  1); }   // A backward, B forward (spins in place)
void turnRight()  { drive( 1, -1); }   // A forward, B backward (spins in place)
void stopMotors() { drive( 0,  0); }
