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
int Clockwise[4];             // Arrays to store whether or not there are obstacles in the four angular positions at which measurements are taken during each sweep
int counterClockwise[4];      // These become 1 if there is an obstacle
int obstaclePresent[4];       // Obstacles are registered if they show up in both clockwise and counterclockwise sweeps in the same position
int servoAngle;               // The servo angle when the distance was measured. This is used to determine the timeout that sets the maximum distance of obstacles

// *** Accelerometer I2C address ***
const int MPU = 0x68;

// *** Encoder for distance measurement ***
#define ENCODER_PIN  3                       // interrupt pin
unsigned long ticks = 0;

// *** Initialize three instances of servos ***
#define HOOK1 8
#define HOOK2 17
#define SERVO_SENSOR 7
ServoTimer2 hookServo1;
ServoTimer2 hookServo2;     
ServoTimer2 sensorServo;

// *** Variables for the motor driver ***

#define leftForward 10      // Control pins for the left motor (PWM)
#define leftReverse 9
#define rightForward 5      // Control pins for the right motor (PWM)
#define rightReverse 6
int drivingSpeed = 170;
int reverseSpeed = 100;
int slightTurnSpeed = 90; 
int turnSpeed = 45;
int sharpTurnSpeed = 0;

RH_RF95 rf95(RFM95_CS, RFM95_INT);  

bool isActive = false;          // isActive is true when the rover is in active (driving or collecting soil) mode

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
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);

  // *** Ultrasonic sensor setup ***
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {

  int servoMillis = 0;                     // this is a counter that will be used to move the sensor servo based on the millis function. It is reset to zero after every cycle.
  long lastServoMillis = 0;
  int servoAngle = 0;
  int totalDistance = 10000;               // total distance that the rover needs to travel. increases each time the rover turns

  if (handleState()) {                     //  returns true if wakeup message is received. Also handles test signal and sends reply to ground control
    while (ticks < totalDistance) {        // Change this number to the actual number of ticks that represents 10+ feet. This is the driving mode. After exiting this loop the rover begins to dig.
                                           // All driving activity takes places within this while loop
      long currentMillis = millis();
      int previousMillis = 0;
      if (currentMillis - previousMillis >= 10000) {      // Check orientation every 10 seconds and flip if needed
        previousMillis = currentMillis;
        if (getOrientation <= -5) {
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
        servoAngle = map(servoMillis, 0, 500, 1200, 1800);  // 1200 to 1800 microseconds are the pulse widths for the scanning angle. This scans left to right.
        sensorServo.write(servoAngle);
      }
      else if (servoMillis > 500 && servoMillis <= 1000) {
        servoAngle = map(servoMillis, 500, 1000, 1800, 1200);
        sensorServo.write(servoAngle);                          // scans right to left
      }
      if (servoMillis == 60) {                               // while the servo is turning, the ultrasonic sensor takes distance measurements at specific angles. four measurements are taken during each sweep, for eight total measurements per cycle.
        Clockwise[0] = obstacleCheck(servoMillis);           // arrays Clockwise and counterClockwise store distance information. 1 if obstacle exists at that angle and 0 if no obstacle exists
      }
      else if (servoMillis == 187) {
        Clockwise[1] = obstacleCheck(servoMillis);           // these values of the servoMillis variable correspond to specific angles at which we want to take distance measurements to detect obstacles
      } 
      else if (servoMillis == 314) {
        Clockwise[2] = obstacleCheck(servoMillis);         
      }
      else if (servoMillis == 441) {
        Clockwise[3] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 559) {
        counterClockwise[0] = obstacleCheck(servoMillis);   
      }
      else if (servoMillis == 686) {
        counterClockwise[1] = obstacleCheck(servoMillis);     
      }
      else if (servoMillis == 813) {
        counterClockwise[2] = obstacleCheck(servoMillis);
      }
      else if (servoMillis == 940) {
        counterClockwise[3] = obstacleCheck(servoMillis);
      }

      if (Clockwise[0] && counterClockwise[3]) 
        obstaclePresent[0] = 1;      // Obstacles only register as true if they show up in both clockwise and counterclockwise scans
      else 
        obstaclePresent[0] = 0;
      if (Clockwise[1] && counterClockwise[2]) 
        obstaclePresent[1] = 1;
      else 
        obstaclePresent[1] = 0;
      if (Clockwise[2] && counterClockwise[1]) 
        obstaclePresent[2] = 1;
      else 
        obstaclePresent[2] = 0;
      if (Clockwise[3] && counterClockwise[0]) 
        obstaclePresent[3] = 1;
      else 
        obstaclePresent[3] = 0;

      // Use the above information about obstacles to decide how to drive

      if (!(obstaclePresent[0] | obstaclePresent[1] | obstaclePresent[2] | obstaclePresent[3])) {          // if there are no obstacles
        driveStraight();
      }                         // there is no default "drive straight" mode. The rover only drives straight if this specific condition is met
      else if (obstaclePresent[0] && (obstaclePresent[1] | obstaclePresent[2] | obstaclePresent[3])) {      // these cover most of the possible combinations of obstacles
        slightRight();
      }
      else if (obstaclePresent[1] == 1 && !(obstaclePresent[2] | obstaclePresent[3])) {
        turnRight();
      }
      else if (obstaclePresent[3] == 1 && !(obstaclePresent[0] | obstaclePresent[1] | obstaclePresent[2])) {
        slightLeft();
      }
      else if (obstaclePresent[2] == 1 && !(obstaclePresent[0] | obstaclePresent[1])) {
        turnLeft();
      }
      else if ((obstaclePresent [1] && obstaclePresent[2]) && !(obstaclePresent[0] | obstaclePresent[3])) {
       turnRight();
      }
      else if ((obstaclePresent[0] && obstaclePresent[1] && obstaclePresent[2]) && !obstaclePresent[3]) {
        turnRight();
      }
      else if ((obstaclePresent[1] && obstaclePresent[2] && obstaclePresent[3]) && !obstaclePresent[0]) {
        turnLeft();
      }
      else if (obstaclePresent[0] && obstaclePresent[3]) {
        sharpLeft();
      }
      else if (obstaclePresent[0]) {         // if any combinations are missed, at least one of these will trigger during each loop and the rover will take appropriate action
        slightRight();
      }
      else if (obstaclePresent[3]) {
        slightLeft();
      }
      else if (obstaclePresent[1]) {
        turnRight();
      }
      else if (obstaclePresent[2]) {
        turnLeft();
      }
      // once the counter reaches the equivalent of the total distance required to drive, the code exits out of this while loop.
    }
    // time to collect soil

    if (getOrientation() <= -5) { 
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
    isActive = false;

    while (1) {                // infinite loop where it just sends the end-of-mission signal to ground control every second
      uint8_t done[] = "Done digging! Come get me";
      uint8_t len = sizeof(done);
      rf95.send(done, &len);
      rf95.waitPacketSent();
      delay(1000);
    }
  }
  else {
    Serial.println("Rover on standby");
  }
}

int getOrientation() {

  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  int z;                       //  only reads the acceleration in the z (vertical) direction. Enough to tell orientation

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

  z = AcZ / 1752;           // Dividing the raw value by this number gives the acceleration in m/s2

  return z;
}

void countTicks() {        // Interrupt service routine for the encoder to measure distance travelled
  ticks++;                 // increments ticks
}

bool handleState() {  
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
        isActive = true;
        uint8_t data[] = "Wakeup command received";
        uint8_t len = sizeof(data);
        rf95.send(data, &len);
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        if (getOrientation() <= -5) {        // check orientation immediately after being activated
          flip();
        }
      }
      else if ((char*)buf[0] == 'T') {       // test signal reads "Testing connection"
        isActive = false;
        uint8_t data[] = "Connection test successful";
        uint8_t len = sizeof(data);
        rf95.send(data, &len);
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
      }
      else if ((char*)buf[0] == 'S') {           // Halt command reads "Stop driving"
        isActive = false;
        uint8_t data[] = "Rover in standby";
        uint8_t len = sizeof(data);
        rf95.send(data, &len);
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
      }
      else{
        isActive = false;
      }
    }
  }
  return isActive;
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

    long duration = pulseIn(ECHO_PIN, HIGH, 10000);      // duration in microseconds for the echo to come back. returns zero if no echo is detected
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
  analogWrite(leftForward, drivingSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void reverse(){
  analogWrite(leftForward, 0);
  analogWrite(rightForward, 0);
  analogWrite(leftReverse, reverseSpeed);
  analogWrite(rightReverse, reverseSpeed);
}
void slightLeft(){
  analogWrite(leftForward, slightTurnSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void slightRight(){
  analogWrite(leftForward, drivingSpeed);
  analogWrite(rightForward, slightTurnSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
  
}
void turnLeft(){
  analogWrite(leftForward, turnSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void turnRight(){
  analogWrite(leftForward, drivingSpeed);
  analogWrite(rightForward, turnSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void sharpLeft(){
  analogWrite(leftForward, sharpTurnSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
