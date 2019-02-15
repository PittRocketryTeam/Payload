// Main code for the rover

#include <SPI.h>
#include <RH_RF95.h>           // For Transceiver
#include <Wire.h>              // For accelerometer/gyro
#include <ServoTimer2.h>       // For servo control. The regular servo library disables PWM on pins 9 and 10. 
// This one uses a different timer register that doesn't interfere with those pins.
// *** Pin definitions and constants for RF95 radio ***
#define RFM95_CS 4
#define RFM95_RST 14
#define RFM95_INT 2
#define RF95_FREQ 434.0

// *** Ultrasonic sensor stuff ***
#define TRIG_PIN  20
#define ECHO_PIN  21
int obs1[4];      // The rover uses the distance of obscales on each sector to decide which way to turn
int obs2[4];      // These become 1 if there is an obstacle
int obs[4];
int servoAngle;                  // The current servo angle when the distance was measured

// *** Variables for accelerometer ***
const int MPU = 0x68;
int Orientation;

// *** Encoder for distance measurement ***
#define ENCODER_PIN  3                       // interrupt pin  
unsigned long ticks = 0;

// *** Initialize three instances of servos ***
#define HOOK1 8
#define HOOK2 17
#define SERVO_SENSOR 7
ServoTimer2 hookServo1;
ServoTimer2 hookServo2;          // create three instances of servos
ServoTimer2 sensorServo;

// *** Variables for the motor driver ***

#define A1 10     // Control pins for the right motor (PWM)
#define A2 9
#define B1 5      // Control pins for the left motor (PWM)
#define B2 6
int drivingSpeed = 170;
int reverseSpeed = 100;

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // initialize rf95 class

bool state = false; // state is true when the rover is in active (driving or collecting soil) mode

void setup()
{
  // *** Transceiver Setup ***
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  // *** ISR setup ***
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countTicks, RISING);

  // *** Servos attached to respective pins ***
  pinMode(HOOK1, OUTPUT);
  pinMode(HOOK2, OUTPUT);
  pinMode(SERVO_SENSOR, OUTPUT);

  hookServo1.attach(HOOK1);
  hookServo2.attach(HOOK2);
  sensorServo.attach(SERVO_SENSOR);

  hookServo1.write(600);                   // initial stowed position of arm
  hookServo2.write(2400);
  sensorServo.write(1500);                 // start with the servo in the neutral position

  // *** DC motor controller pins ***
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);

  // *** Ultrasonic sensor setup ***
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
        
  int servoMillis = 0;                     // this is a counter that will be used to move the sensor servo based on the millis function. It is reset to zero after every cycle.
  long lastServoMillis = 0;
  int servoAngle = 0;
      
  if (readState()) {                   //  returns true if wakeup message is received
    while (ticks < 100000) {           // Change this number to the actual number of ticks that represents 10+ feet. This is the driving mode. After exiting this loop the rover begins to dig.

      long currentMillis = millis();
      int previousMillis = 0;
      if (currentMillis - previousMillis >= 10000) {      // Check orientation every 10 seconds and flip if needed
        Orientation = getOrientation();
        previousMillis = currentMillis;
        if (Orientation <= -5) {
          flip();
        }
      }

      // The rover now begins to scan with the Ultrasonic sensor while the servo moves side to side.
      // Distance measurements are taken at 4 discrete instances during each sweep

      servoMillis = millis() - lastServoMillis;            // servoMillis goes from 0 to 1000 and rolls back to 0 every second.
      if (servoMillis >= 1000) {                           // the value of servoMillis is mapped to the servo angle to make it scan
        servoMillis = 0;                                   // 0 to 500 is clockwise and 500 to 1000 is counterclockwise
        lastServoMillis = millis();
      }
      if (servoMillis > 0 && servoMillis <= 500) {
        servoAngle = map(servoMillis, 0, 500, 1200, 1800);          // 1200 to 1800 microseconds are the pulse widths for the scanning angle. This scans left to right.
        sensorServo.write(servoAngle);
      }
      else if (servoMillis > 500 && servoMillis <= 1000) {
        servoAngle = map(servoMillis, 500, 1000, 1800, 1200);
        sensorServo.write(servoAngle);                 // scans right to left
      }
      if (servoMillis == 60) {
        obs1[0] = obstacleCheck(servoMillis);           // arrays obs1 and obs2 store distance information. 1 if obstacle exists at that angle and 0 if no obstacle exists
      }
      else if (servoMillis == 187) {
        obs1[1] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 314) {
        obs1[2] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 441) {
        obs1[3] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 559) {
        obs2[0] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 686) {
        obs2[1] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 813) {
        obs2[2] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 940) {
        obs2[3] = obstacleCheck(servoMillis);
      }

      if (obs1[0] && obs2[3]) obs[0] = 1;      // Obstacles only register as true if they show up in both clockwise and counterclockwise scans
      else obs[0] = 0;
      if (obs1[1] && obs2[2]) obs[1] = 1;
      else obs[1] = 0;
      if (obs1[2] && obs2[1]) obs[2] = 1;
      else obs[2] = 0;
      if (obs1[3] && obs2[0]) obs[3] = 1;
      else obs[3] = 0;

      // Use the above information to decide how to drive

      if (!(obs[0] | obs[1] | obs[2] | obs[3])) {          // if there are no obstacles
        driveStraight();
      }
      else if (obs[0] = 1 && !(obs[1] | obs[2] | obs[3])) {
        slightRight();
      }
      else if (obs[1] = 1 && !(obs[2] | obs[3])) {
      turnRight();
      }
      else if (obs[3] = 1 && !(obs[0] | obs[1] | obs[2])) {
      slightLeft();
      }
      else if (obs[2] = 1 && !(obs[0] | obs[1])) {
      turnLeft();
      }
      else if ((obs [1] && obs[2]) && !(obs[0] | obs[3])) {
      turnRight();
      }
      else if ((obs[0] && obs[1] && obs[2]) && !obs[3]) {
      turnRight();
      }
      else if ((obs[1] && obs[2] && obs[3]) && !obs[0]) {
      turnLeft();
      }
      else if (obs[0] && obs[3]) {
      sharpLeft();
      }
      else if (obs[0]) {
      slightRight();
      }
      else if (obs[3]) {
      slightLeft();
      }
      else if (obs[1]) {
      turnRight();
      }
      else if (obs[2]) {
      turnLeft();
      }
      // once the counter reaches the equivalent of around 15 ft, the code exits out of this while loop.
    }
    // time to collect soil

    Orientation = getOrientation();
    if (Orientation <= -5) {
      flip();
    }
    for (int i = 0; i < 5; i++) {
      deployArm();
      delay (1000);
      reverse();
      delay(10000);
      retractArm();               // simple series of events separated by delays
      delay(500);
      driveStraight();
      delay(2000);
      deployArm();
      reverse();
      delay(10000);
      retractArm();
    }
    delay(1000);
    state = false;

    while (1) {                // infinite loop where it just sends the end-of-mission signal to ground control every second
      uint8_t done[] = "Done digging! Come get me";
      rf95.send(done, sizeof(done));
      rf95.waitPacketSent();
      delay(1000);
    }
  }
  else {
    Serial.println("Rover in standby");
  }
}
int getOrientation() {

  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  int Acc[3];
  int x, y, z;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 12, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  x = AcX / 1752;
  y = AcY / 1752;
  z = AcZ / 1752;

  Acc[0] = x;
  Acc[1] = y;
  Acc[2] = z;

  return z;
}

void countTicks() {        // Interrupt service routine for the encoder to measure distance travelled
  ticks++;                 // literally just counts ticks.
}

bool readState() {
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      delay(10);
      if ((char*)buf[0] == 'W') {        // wakeup signal reads "Wake up Wall E!"
        state = true;
        uint8_t data[] = "Wakeup command received";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        Orientation = getOrientation();   // check orientation immediately after being activated
        if (Orientation <= -5) {
          flip();
        }
      }
      else if ((char*)buf[0] == 'T') {   // test signal reads "Testing connection"
        state = false;
        uint8_t data[] = "Connection test successful";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
      }
      else if ((char*)buf[0] == 'S') {           // Halt command reads "Stop driving"
        state = false;
        uint8_t data[] = "Rover in standby";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
      }
      else{
        state = false;
      }
    }
  }
  return state;
}
void deployArm() {
  hookServo1.write(2400);
  hookServo2.write(600);
}
void retractArm() {
  hookServo1.write(600);
  hookServo2.write(2400);
}
void flip() {
  hookServo1.write(2400);
  hookServo2.write(600);
  delay(1000);

  hookServo1.write(600);
  hookServo2.write(2400);

  delay(1000);
}

int obstacleCheck(int angle) {
  if (angle < 250 | angle > 750) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 10000);
    if (duration > 0) return 1;
    else return 0;
  }
  else {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 15000);
    if (duration > 0) return 1;
    else return 0;
  }
}

void driveStraight(){
  analogWrite(A1, drivingSpeed);
  analogWrite(B1, drivingSpeed);
}
void reverse(){
  analogWrite(A2, reverseSpeed);
  analogWrite(B2, reverseSpeed);
}
void slightLeft(){
  analogWrite(A1, 100);
  analogWrite(B1, 170);
}
void slightRight(){
  analogWrite(A1, 170);
  analogWrite(B1, 100);
}
void turnLeft(){
  analogWrite(A1, 50);
  analogWrite(B1, 170);
}
void turnRight(){
  analogWrite(A1, 170);
  analogWrite(B1, 50);
}
void sharpLeft(){
  analogWrite(A1, 0);
  analogWrite(B1, 170);
}
