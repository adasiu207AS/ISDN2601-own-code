#include <ESP32Servo.h>

// ==============================================================================
// GPIO CONFIGURATION
// ==============================================================================
#define START_BUTTON     25  // Start button (active LOW, connected to GND)

// Motor Driver Pins (Differential Drive)
#define MOTOR_L_POS      17  // Left motor forward terminal (GPIO 17)
#define MOTOR_L_NEG      16  // Left motor reverse terminal (GPIO 16)
#define MOTOR_R_POS      26  // Right motor forward terminal (GPIO 26)
#define MOTOR_R_NEG      27  // Right motor reverse terminal (GPIO 27)

// IR Sensors (Switch set to 'D' - GPIO 14, 36, 39)
#define IR_L_PIN         14  // Left IR sensor
#define IR_M_PIN         36  // Middle IR sensor
#define IR_R_PIN         39  // Right IR sensor

// Ultrasonic (US) Sensor
#define TRIG_PIN         33  // US trigger pin
#define ECHO_PIN         32  // US echo pin

// Arm Servos (Front: 19, Middle: 18, Back: 13)
#define SERVO_1_PIN      19  // Front servo (verify physical joint & angle)
#define SERVO_2_PIN      18  // Middle servo (verify physical joint & angle)
#define SERVO_3_PIN      13  // Back servo (verify physical joint & angle)

// ==============================================================================
// SERVO DEFINITIONS & ANGLES (Calibrate on your assembled arm)
// ==============================================================================
Servo myServo1;
Servo myServo2;
Servo myServo3;

#define SERVO1_CLOSE     60
#define SERVO1_OPEN       0

#define SERVO2_UP        110
#define SERVO2_DOWN       70

#define SERVO3_UP        110
#define SERVO3_DOWN       70

// ==============================================================================
// TUNING PARAMETERS
// ==============================================================================
#define MOTOR_SPEED_L    110  // Left motor forward PWM (0 - 255)
#define MOTOR_SPEED_R    110  // Right motor forward PWM (0 - 255)
#define TURN_SPEED       110  // Turning PWM speed

#define START_FORWARD_MS  1000 // Clear start line after button press
#define TURN_45_MS         150 // Small steering correction duration
#define TURN_90_MS        1600 // T-junction 90-degree turn duration (tune on track)
#define OBJECT_ADVANCE_MS  400 // Advance from Grab code to object (~10 cm, tune)
#define RETURN_CLEAR_MS   1200 // Drive past start/finish line on return (tune)

#define OBSTACLE_STOP_CM  15.0 // Stop distance before station wall/box

#define DIR_NONE          0
#define DIR_LEFT          1
#define DIR_RIGHT         2

// Set to 1 if IR reads HIGH on BLACK, or 0 if inverted (LOW on BLACK)
#define BLACK             1
#define WHITE             0

// ==============================================================================
// STATE VARIABLES (FSM)
// ==============================================================================
enum RobotState {
  STATE_FOLLOW_TO_GRAB,
  STATE_GRABBING,
  STATE_FOLLOW_TO_TURN_CODE,
  STATE_FOLLOW_TO_T_JUNCTION,
  STATE_TURNING_90,
  STATE_APPROACH_STATION,
  STATE_DROPPING,
  STATE_RETURNING,
  STATE_DONE
};

RobotState currentState = STATE_FOLLOW_TO_GRAB;
int remembered_turn = DIR_NONE;

// Forward declarations
int readIR(int pin);
float measureDistance();
void motorStop();
void motorForward(int speedL, int speedR);
void motorForward();
void forward(int duration_ms);
void turnLeft45();
void turnRight45();
void turnLeft90();
void turnRight90();

// ==============================================================================
// SERVO FUNCTIONS
// ==============================================================================
void initServos() {
  ESP32PWM::allocateTimer(0);

  myServo1.setPeriodHertz(50);
  myServo1.attach(SERVO_1_PIN, 500, 2500);

  myServo2.setPeriodHertz(50);
  myServo2.attach(SERVO_2_PIN, 500, 2500);

  myServo3.setPeriodHertz(50);
  myServo3.attach(SERVO_3_PIN, 500, 2500);
}

void setServo(Servo& servo, int angle) {
  servo.write(angle);
}

// Pre-start 60 deg reference pose required by rubric
void armInit() {
  Serial.println("[0] Initializing Arm to 60 deg reference pose");
  setServo(myServo1, 60);
  delay(200);
  setServo(myServo2, 100);
  delay(200);
  setServo(myServo3, 110);
  delay(200);
}

void armUp() {
  setServo(myServo1, SERVO1_OPEN);
  delay(300);
  setServo(myServo2, SERVO2_UP);
  delay(300);
  setServo(myServo3, SERVO3_UP);
  delay(300);
  Serial.println("Arm is UP");
}

void armDown() {
  setServo(myServo2, SERVO2_DOWN);
  delay(300);
  setServo(myServo3, SERVO3_DOWN);
  delay(300);
  setServo(myServo1, SERVO1_CLOSE);
  delay(500);
  Serial.println("Arm is DOWN");
}

void grabObject() {
  motorStop();
  armDown();
  setServo(myServo1, SERVO1_CLOSE);
  delay(400);
  armUp();
}

void dropObject() {
  motorStop();
  armDown();
  setServo(myServo1, SERVO1_OPEN);
  delay(400);
  armUp();
}

// ==============================================================================
// MOTOR CONTROL FUNCTIONS
// ==============================================================================
void motorStop() {
  analogWrite(MOTOR_L_POS, 0);
  analogWrite(MOTOR_L_NEG, 0);
  analogWrite(MOTOR_R_POS, 0);
  analogWrite(MOTOR_R_NEG, 0);
  digitalWrite(MOTOR_L_POS, LOW);
  digitalWrite(MOTOR_L_NEG, LOW);
  digitalWrite(MOTOR_R_POS, LOW);
  digitalWrite(MOTOR_R_NEG, LOW);
}

void motorForward(int speedL, int speedR) {
  analogWrite(MOTOR_L_POS, speedL);
  analogWrite(MOTOR_L_NEG, 0);
  analogWrite(MOTOR_R_POS, speedR);
  analogWrite(MOTOR_R_NEG, 0);
}

void motorForward() {
  motorForward(MOTOR_SPEED_L, MOTOR_SPEED_R);
}

void forward(int duration_ms) {
  Serial.printf("Moving forward for %d ms\n", duration_ms);
  motorForward();
  delay(duration_ms);
}

void turnLeft45() {
  Serial.println("Turn left 45 degrees");
  analogWrite(MOTOR_L_POS, 0);
  analogWrite(MOTOR_L_NEG, TURN_SPEED);
  analogWrite(MOTOR_R_POS, TURN_SPEED);
  analogWrite(MOTOR_R_NEG, 0);
  delay(TURN_45_MS);
  motorStop();
}

void turnRight45() {
  Serial.println("Turn right 45 degrees");
  analogWrite(MOTOR_L_POS, TURN_SPEED);
  analogWrite(MOTOR_L_NEG, 0);
  analogWrite(MOTOR_R_POS, 0);
  analogWrite(MOTOR_R_NEG, TURN_SPEED);
  delay(TURN_45_MS);
  motorStop();
}

void turnLeft90() {
  Serial.println("Turn left 90 degrees");
  analogWrite(MOTOR_L_POS, 0);
  analogWrite(MOTOR_L_NEG, TURN_SPEED);
  analogWrite(MOTOR_R_POS, TURN_SPEED);
  analogWrite(MOTOR_R_NEG, 0);
  delay(TURN_90_MS);
  motorStop();
}

void turnRight90() {
  Serial.println("Turn right 90 degrees");
  analogWrite(MOTOR_L_POS, TURN_SPEED);
  analogWrite(MOTOR_L_NEG, 0);
  analogWrite(MOTOR_R_POS, 0);
  analogWrite(MOTOR_R_NEG, TURN_SPEED);
  delay(TURN_90_MS);
  motorStop();
}

// ==============================================================================
// SENSOR FUNCTIONS
// ==============================================================================
int readIR(int pin) {
  int val = digitalRead(pin);
  return (val == HIGH) ? BLACK : WHITE;
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

void followLineStep(int ir_L, int ir_M, int ir_R) {
  if (ir_L == WHITE && ir_M == BLACK && ir_R == WHITE) {
    motorForward();
  } else if (ir_L == BLACK && ir_M == WHITE && ir_R == WHITE) {
    turnLeft45();
  } else if (ir_L == WHITE && ir_M == WHITE && ir_R == BLACK) {
    turnRight45();
  } else if (ir_L == WHITE && ir_M == WHITE && ir_R == WHITE) {
    motorForward();
  } else {
    motorStop();
  }
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(START_BUTTON, INPUT_PULLUP);

  pinMode(MOTOR_L_POS, OUTPUT