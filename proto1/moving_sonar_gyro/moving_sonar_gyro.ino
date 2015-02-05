
/* Robot Caterpillar
   2015-01-19 Arjen Lentz
*/

#include <Servo.h>  // for Servos
#include <Wire.h>   // for Accel/Gyro I2C module


#define NUM_SERVOS  8

// which are which
#define HEAD  0
#define LEG1  1
#define TURN1 2
#define LEG2  3
#define TURN2 4
#define LEG3  5
#define TURN3 6
#define TAIL  7

#define POS_MIN     90
#define POS_MAX    180
#define POS_REST   ((POS_MAX - POS_MIN) / 2) + POS_MIN
#define POS_CYCLE  (POS_MAX - POS_MIN) * 2

Servo servo[NUM_SERVOS];  // create servo control objects
int servo_pins[NUM_SERVOS] = { 5, 6, 7, 8, 9, 10, 11, 12 };
int servo_pos[NUM_SERVOS];
boolean servo_dir[NUM_SERVOS];



#define SONAR_PIN               4
// Scientific baseline: speed of sound is 340 m/s (29 usecs/cm)
#define SONAR_USECS_PER_CM     29
#define SONAR_RANGE_LIMIT    2000    // = 20 metres
#define SONAR_TIMEOUT  (SONAR_RANGE_LIMIT * SONAR_USECS_PER_CM)


/* Sonar ping - obstacle/distance detection
   based on http://www.arduino.cc/en/Tutorial/Ping
   created 3 Nov 2008 by David A. Mellis
   modified 30 Aug 2011 by Tom Igoe
   This example code is in the public domain.

   Using HC-SR04 module
*/
int sonar_ping (int pin)
{
  long usecs;
  int cm;

  // Sonar ping is triggered by a HIGH pulse of >=2 microsecs.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the sonar module:
  // HIGH pulse, the duration translates to the time (in microseconds)
  // from the sending of the ping to the reception of its echo.
  pinMode(pin, INPUT);
  // PulseIn() has a default 1 second timeout
  // on timeout, 0 is returned, otherwise time in usecs
  usecs = pulseIn(pin, HIGH, SONAR_TIMEOUT);

  // convert the time into a distance
  // The speed of sound is 340 m/s (29 microseconds per centimeter).
  // The ping travels out and back, so we need to divide by 2
  cm = (usecs / SONAR_USECS_PER_CM) / 2;

  return (cm);
}



/* MPU-6050 Accelerometer/Gyroscope/Thermometer I2C module
   based on sample code by JohnChi, August 17, 2014, Public Domain

   this code doesn't suffice for being completely upside down, but that's ok for now
*/

#define MPU 0x68    // I2C address of the MPU-6050

#define FLUX_THRESHOLD      1000
#define ROLL_THRESHOLD     10000
#define STRAIGHT_THRESHOLD  2000

enum { GYRO_FLUX, GYRO_STRAIGHTUP, GYRO_LEFTSIDE, GYRO_RIGHTSIDE, GYRO_UPSIDEDOWN };


void init_gyro(void)
{
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}


int read_gyro_roll(void)
{
  static int16_t prevAcX = 0, prevAcY = 0;
  int16_t AcX, AcY, state;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);    // we're not finished yet

  Wire.requestFrom(MPU, 4, true); // request a total of 4 registers
  AcX = (Wire.read() << 8) | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = (Wire.read() << 8) | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  if (abs(AcX - prevAcX) > FLUX_THRESHOLD || abs(AcY - prevAcY) > FLUX_THRESHOLD)
    state = GYRO_FLUX;
  else if (AcX < 0)
    state = GYRO_UPSIDEDOWN;
  else if (AcY < -ROLL_THRESHOLD)
    state = GYRO_LEFTSIDE;
  else if (AcY > ROLL_THRESHOLD)
    state = GYRO_RIGHTSIDE;
  else
    state = GYRO_STRAIGHTUP;

  prevAcX = AcX;
  prevAcY = AcY;

  return (state);
}


#if 0
// note: this only works on a relatively flat floor
// if caterpillar is on an incline, it won't work so well
int read_gyro_lookstraight(void)
{
  static int16_t prevAcZ = 0;
  int16_t AcZ, state;

  Wire.beginTransmission(MPU);
  Wire.write(0x3F);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);    // we're not finished yet

  Wire.requestFrom(MPU, 2, true); // request a total of 4 registers
  AcZ = (Wire.read() << 8) | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  if (abs(AcZ - prevAcZ) > FLUX_THRESHOLD)
    state = false;    // too much movement
  else if (abs(AcZ) < STRAIGHT_THRESHOLD)
    state = true;     // looking relatively straight ahead
  else
    state = false;    // too far up or down

  prevAcZ = AcZ;

  Serial.print("Lookstraight state = ");
  Serial.println(state);

  return (state);
}
#endif


void init_walk_forward(void)
{
  servo_pos[LEG1] = 140; servo_dir[LEG1] = true;
  servo_pos[LEG2] = 110; servo_dir[LEG2] = true;
  servo_pos[LEG3] = 180; servo_dir[LEG3] = false;
}



// just reverse walking direction
// not moving servos directly
void walk_reverse(void)
{
  servo_dir[LEG1] = !servo_dir[LEG1];
  servo_dir[LEG2] = !servo_dir[LEG2];
  servo_dir[LEG3] = !servo_dir[LEG3];
}


// move one leg
void walk_leg(int leg)
{
  // set new position
  servo[leg].write(servo_pos[leg]);

  // adjust for next position
  if (servo_dir[leg]) {
    if (++servo_pos[leg] >= POS_MAX)
      servo_dir[leg] = !servo_dir[leg];
  }
  else {
    if (--servo_pos[leg] <= POS_MIN)
      servo_dir[leg] = !servo_dir[leg];
  }
}


// move all legs for an entire cycle
void walk_legs(int num_cycles)
{
  for (int i = 0; i < POS_CYCLE * num_cycles; i++) {
    walk_leg(LEG1);
    walk_leg(LEG2);
    walk_leg(LEG3);

    delay(5);  // waits a bit for the servos to reach the position
  }
}


void tail_up(void)
{
  int i;

  for (i = servo_pos[TAIL]; i > POS_MIN; i--) {
    servo[TAIL].write(servo_pos[TAIL] = i);
    delay(20);
  }
}


void tail_straight(void)
{
  int i;

  for (i = servo_pos[TAIL]; i < POS_REST; i++) {
    servo[TAIL].write(servo_pos[TAIL] = i);
    delay(20);
  }
}


void turn_head(boolean right)
{
  int i;
  int dir = right ? -1 : 1;
  int topos = right ? POS_MIN : POS_MAX;

  for (i = servo_pos[HEAD]; i != topos; i += dir) {
    servo[HEAD].write(servo_pos[HEAD] = i);
    delay(20);
  }
}


void turn_head_straight(void)
{
  int i;
  int dir;

  if (servo_pos[HEAD] > POS_REST)  // are we turned left
    dir = -1;
  else  // or right
    dir = 1;

  for (i = servo_pos[HEAD]; i != POS_REST; i += dir) {
    servo[HEAD].write(servo_pos[HEAD] = i);
    delay(20);
  }
}


void turn_body(boolean right)
{
  int i;
  int dir = right ? -1 : 1;
  int topos = right ? POS_MIN + 20: POS_MAX - 20;

  for (i = POS_REST; i != topos; i += dir) {
    servo[TURN1].write(servo_pos[TURN1] = i);
    servo[TURN2].write(servo_pos[TURN2] = i);
    servo[TURN3].write(servo_pos[TURN3] = i);
    delay(20);
  }
}


void turn_body_straight(void)
{
  int i;
  int dir;

  if (servo_pos[TURN1] > POS_REST)  // are we turned right
    dir = -1;
  else  // or left
    dir = 1;

  for (i = servo_pos[TURN1]; i != POS_REST; i += dir) {
    servo[TURN1].write(servo_pos[TURN1] = i);
    servo[TURN2].write(servo_pos[TURN2] = i);
    servo[TURN3].write(servo_pos[TURN3] = i);
    delay(20);
  }
}


void setup()
{
  // debug
  Serial.begin(9600);
  Serial.println("Caterpillar! by Arjen");

  // Servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo[i].attach(servo_pins[i]);
    servo[i].write(servo_pos[i] = POS_REST);      // straight stretch
    servo_dir[i] = true;
    delay(200);
  }

  // initialise walk
  init_walk_forward();

  // Sonar module doesn't require initialisation

  // Accel/Gyro module
  Wire.begin();

  Serial.println("Startup rest");
  delay(3000);
}



void loop()
{
  int i;
  int cm = sonar_ping(SONAR_PIN);

  // check ahead

  Serial.print("Sonar ping ");
  Serial.print(cm);
  Serial.println(" cm");

  // do we have a valid ping
  // and we're really close to hitting something?
  if (cm > 0 && cm < 10) {
    int left_cm, right_cm;

    tail_up();

    // take evasive action
    walk_reverse();
    do {
      Serial.println("Reversing!");
      walk_legs(3);

      // look left
      Serial.println("Looking left");
      turn_head(false);
      delay(200);
      left_cm = sonar_ping(SONAR_PIN);

      // look right
      Serial.println("Looking right");
      turn_head(true);
      delay(200);
      right_cm = sonar_ping(SONAR_PIN);

      // head straight!
      turn_head_straight();
      delay(200);

      tail_straight();
      tail_up();
      delay(200);

      // keep reversing until we can see a clear path left or right
    } while (left_cm < 10 && right_cm < 10);

    if (left_cm > right_cm) {
      Serial.println("Turning left");
      turn_body(false);
    }
    else {
      Serial.println("Turning right");
      turn_body(true);
    }

    // we can move forward again
    Serial.println("Forward turn");
    walk_reverse();
    walk_legs(10);

    Serial.println("Straight!");
    turn_body_straight();
    
    tail_straight();
  }
  else              // no obstacles
    walk_legs(1);   // do a step
}

