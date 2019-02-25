// Ground control sketch for payload

// There are three signals that can be sent to the rover through serial.
// 'T' sends a test packet and waits for a reply to confirm that the connection is good.
// 'A' sends a wakeup call to activate the rover and begin driving. The rover sends a reply if it received the correct packet.
// 'H' sends a halt command to stop driving and switch to standby
// When confirmation is received that the rover has been activated, ground control keeps the receiver on and waits for the rover
// to send the end-of-mission signal.

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4                     // Pin declarations for the CS, reset and interrupt pin. Pin 14 is Analog pin 0. These can be changed.
#define RFM95_RST 14
#define RFM95_INT 2

#define RF95_FREQ 434.0                // Set frequency to 434.0 MHz which is 1 MHz away from the GPS transciever frequency.

RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool state = false;                    // state is true if the rover has been activated and is driving.

void setup()
{ 
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  delay(100);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);             // manual reset
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);                // Set max transmitting power
  digitalWrite(10, LOW);
}

void loop()
{

  char test[] = "Test";
  char wakeUp[] = "Wake";
  char halt[] = "Stop Driving";

  while (Serial.available()) {


    char c = Serial.read();       // Read input from serial (ground computer)
                                
    if (c == 'T') {

      Serial.println("Sending test packet to rover");
      delay(10);
      rf95.send((uint8_t *)test, sizeof(test));

      rf95.waitPacketSent();

      uint8_t testBuffer[RH_RF95_MAX_MESSAGE_LEN];          // buffer to hold reply from rover
      uint8_t testBufferLen = sizeof(testBuffer);

      Serial.println("Waiting for reply...");
      delay(10);
      if (rf95.waitAvailableTimeout(1000))                  // Starts receiver and returns true if reply received within 1000 ms
      {
        if (rf95.recv(testBuffer, &testBufferLen))
        {
          Serial.print("Got reply: ");
          Serial.println((char*)testBuffer);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
        }
        else
        {
          Serial.println("Failed to read reply");
        }
      }
      else
      {
        Serial.println("Rover could not be reached. Please try again.");
      }
    }
    else if (c == 'A') {                    // If signal to activate rover is received through serial

      Serial.println("Sending wakeup command to rover..");
      delay(10);
      rf95.send((uint8_t *)wakeUp, sizeof(wakeUp));

      rf95.waitPacketSent();

      uint8_t activateBuffer[RH_RF95_MAX_MESSAGE_LEN];                  // buffer to hold reply from rover
      uint8_t activateBufferLen = sizeof(activateBuffer);

      Serial.println("Waiting for reply..."); delay(10);
      if (rf95.waitAvailableTimeout(1000))                       // Starts receiver and returns true if reply received within 1000 ms
      {
        if (rf95.recv(activateBuffer, &activateBufferLen))
        {
          Serial.print("Got reply: ");
          Serial.println((char*)activateBuffer);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
          bool state = true;                                     // Rover has been activated and groundcontrol will start
          Serial.println("Waiting for end of mission signal.");  // scanning for any received messages.
        }
        else
        {
          Serial.println("Failed to read reply. Rover may have activated.");    // This means a signal was received but could not be read.
        }                                                                       // It's possible that the rover still got the wakeup command
      }
      else
      {
        Serial.println("Rover could not be reached. Please try again.");
      }
    }
    else if (c == 'H'){
      Serial.println("Sending halt command to rover..");
      delay(10);
      rf95.send((uint8_t *)halt, sizeof(halt));

      rf95.waitPacketSent();

      uint8_t haltBuffer[RH_RF95_MAX_MESSAGE_LEN];                  // buffer to hold reply from rover
      uint8_t haltBufferLen = sizeof(haltBuffer);

      Serial.println("Waiting for reply..."); delay(10);
      if (rf95.waitAvailableTimeout(1000))                       // Starts receiver and returns true if reply received within 1000 ms
      {
        if (rf95.recv(haltBuffer, &haltBufferLen))
        {
          Serial.print("Got reply: ");
          Serial.println((char*)haltBuffer);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);                     
        }
        else
        {
          Serial.println("Failed to read reply.");    // This means a signal was received but could not be read.
        }                                                                       
      }
      else
      {
        Serial.println("Rover could not be reached. Please try again.");
      }
    }
    else {
      Serial.read();                  // Empty the serial input buffer if anything other than the three commands are received.
    }
  }

  if (state) {                                             // if rover has been activated
    if(rf95.available()){
      uint8_t endBuffer[RH_RF95_MAX_MESSAGE_LEN];          // Buffer to store end-of-mission message received from rover.
      uint8_t endBufferLen = sizeof(endBuffer);

      if (rf95.recv(endBuffer, &endBufferLen))             // Keep scanning the receiver for any message received.
      {
        Serial.println("Received message from rover:");
        Serial.println((char*)endBuffer);
      }
    }
  }
}
