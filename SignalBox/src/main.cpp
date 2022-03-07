#include <Arduino.h>

#include <myWiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "myEEPROM.h"

#define softwareVersion 1.10

const int udpPort = 44444;
WiFiUDP udp;
uint8_t txBuffer[100] = "";
uint8_t rxBuffer[100] = "";
IPAddress lastIP;

uint8_t sensorStart[16] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150}; // default to 2m start (150cm + 50cm)
uint8_t sensorSpan[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};  // default to 1m  span -> detection range from 2m to 3m

// IPAddress sensorAddress[16];

bool sensorActive[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
bool seenActive[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

hw_timer_t *fastTimer = NULL;
hw_timer_t *slowTimer = NULL;

// EEPROM
myEEPROM eeprom;

// defines variables
bool ledState;
bool detected;
bool sendAddress;
bool clearSensorActive;
bool clearPressed = false;
bool stopPressed = false;
bool isMaster;
bool eepromAvailable = false;

uint16_t portA;
uint16_t portB;

bool manualState[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; // indicates manual stop switch is active

// int demoSteps = 7; // the default number of lights on the board for demo mode

#define wifiActive 4
#define signalsActive 2
#define stopButton 15
#define clearButton 16
#define master 34
#define uartReset 17
#define espReset

#define EEPROMtestaddr 32
#define EEPROMstartaddr 0
#define EEPROMspanaddr 128



void timerInterrupt()
{
  sendAddress = true;
}

void slowTimerInterrupt()
{
  clearSensorActive = true;
}

void writePortA(uint16_t value)
{
  Wire.beginTransmission(0x20); // first 23017
  Wire.write(0x12);             // Bank A
  uint8_t aByte = value & 0xff;
  Wire.write(aByte);
  Wire.endTransmission();
  Wire.beginTransmission(0x21); // second 23017
  Wire.write(0x12);             // Bank A
  aByte = value >> 8;
  Wire.write(aByte);
  Wire.endTransmission();
}

void writePortB(uint16_t value)
{
  Wire.beginTransmission(0x20); // first 23017
  Wire.write(0x13);             // Bank B
  uint8_t aByte = value & 0xff;
  Wire.write(aByte);
  Wire.endTransmission();
  Wire.beginTransmission(0x21); // second 23017
  Wire.write(0x13);             // Bank B
  aByte = value >> 8;
  Wire.write(aByte);
  Wire.endTransmission();
}

void ledRed(int sensor)
{
  uint16_t value = 0x0001 << sensor;
  // Serial.print("bit pattern:");
  // Serial.print(value, BIN);
  // Serial.print(" for sensor:");
  // Serial.println(sensor);
  Serial.print("RED on for sensor:");
  Serial.println(sensor);
  portA = portA | value;
  writePortA(portA);
}

void ledGreen(int sensor)
{
  uint16_t value = 0x0001 << sensor;
  value = value ^ 0xFFFF;
  // Serial.print("bit pattern:");
  // Serial.print(value, BIN);
  // Serial.print(" for sensor:");
  // Serial.println(sensor);
  Serial.print("GREEN on for sensor:");
  Serial.println(sensor);
  portA = portA & value;
  writePortA(portA);
}

void occupiedSection(int sensor)
{
  // turns on the yellow led to show train in section
  uint16_t value = 0x0001 << sensor;
  value = value ^ 0xFFFF;
  // Serial.print("bit pattern:");
  // Serial.print(value, BIN);
  // Serial.print(" for sensor:");
  // Serial.println(sensor);
  portB = portB & value;
  writePortB(portB);
  Serial.print("YELLOW on for track:");
  Serial.println(sensor);
}

void clearSection(int sensor)
{
  // turn off the yellow led to show the section is clear
  uint16_t value = 0x0001 << sensor;
  // Serial.print("bit pattern:");
  // Serial.print(value, BIN);
  // Serial.print(" for sensor:");
  // Serial.println(sensor);
  portB = portB | value;
  writePortB(portB);
  Serial.print("YELLOW off for track:");
  Serial.println(sensor);
}

bool compareBuffers(char *buf1, const char *buf2, int num)
{
  if (num == 0)
    return false;
  for (int i = 0; i < num; i++)
  {
    if (buf1[i] != buf2[i])
    {
      return false;
    }
    // Serial.print("buf1:");
    // Serial.print(buf1[i]);
    // Serial.print("  buf2:");
    // Serial.println(buf2[i]);
  }
  return true;
}

bool compareBuffersFrom(char *buf1, const char *buf2, int from, int num)
{
  if (num == 0)
    return false;

  // Serial.print("from:");
  // Serial.print(from);
  // Serial.print("  num:");
  // Serial.println(num);

  for (int i = from; i < (num + from); i++)
  {
    // Serial.print("buf1:");
    // Serial.print(buf1[i]);
    // Serial.print("  buf2:");
    // Serial.println(buf2[i - from]);
    if (buf1[i] != buf2[i - from])
    {
      return false;
    }
  }
  return true;
}

char *getUdpMessage()
{
  udp.parsePacket();
  rxBuffer[0] = 0;
  if (udp.read(rxBuffer, 50) > 0)
  {
    // received a message
    // Serial.print("getUdpMessage->Received: ");
    // Serial.println((char *)buffer);
    lastIP = udp.remoteIP();
  }
  return (char *)rxBuffer;
}

void copyToTxBuffer(const char *str)
{
  memcpy(txBuffer, str, strlen(str));
}

String binaryString(uint16_t value)
{
  char buffer[17] = "0000000000000000";
  for (int i = 0; i < 16; i++)
  {
    uint16_t test = 0x0001 << i;
    if (value & test)
    {
      // bit is a '1'
      buffer[i] = '1';
    }
  }
  return String(buffer);
}

void sendAddressMessage()
{
  // broadcast message to the signals sent every 500ms
  //                    led           Sensor setting
  //                    stat      start distance, span
  //                    |--||------------------------------|
  // "Signalbox Address:AaBb00112233445566778899aabbccddeeff255.255.255.255"
  //  0000000000111111111122222222223333333333444444444455555555556666666666
  //  0123456789012345678901234567890123456789012345678901234567890123456789
  //
  //  sensor start distance = 8bit value (cm) + 50 cm minimum offset
  //  sensor span distance = 8bit value (cm)

  static uint16_t lastPortA = 0;
  static uint16_t lastPortB = 0;
  if (isMaster)
  {
    //  This bradcasts the signalbox IP address if the signalbox is a master
    copyToTxBuffer("Signalbox Address: ");
    uint8_t aByte;
    aByte = portA >> 8;
    txBuffer[18] = aByte; // put in the bit pattern for the lights (bits 4-7)
    aByte = portA & 0xff;
    txBuffer[19] = aByte; // put in the bit pattern for the lights (bits 0-3)
    aByte = portB >> 8;
    txBuffer[20] = aByte; // put in the bit pattern for the lights (bits B-F)
    aByte = portB & 0xFF;
    txBuffer[21] = aByte; // put in the bit pattern for the lights (bits 8-A)

    // add the sensor settings
    for (int i = 0; i <= 15; i++)
    {
      txBuffer[22 + (i * 2)] = sensorStart[i];
      txBuffer[23 + (i * 2)] = sensorSpan[i];
    }

    const char *adr = WiFi.localIP().toString().c_str();
    int adrLen = strlen(adr);
    // Serial.println(adrLen);
    memcpy(&txBuffer[54], adr, adrLen);
    // Serial.println((char *)buffer);
    udp.beginPacket("255.255.255.255", udpPort);
    udp.write(txBuffer, 69);
    udp.endPacket();
    sendAddress = false;
    if (lastPortA != portA || lastPortB != portB)
    {
      String portAString = binaryString(portA);
      String portBString = binaryString(portB);
      Serial.print("Signalbox Address Broadcast with portA:");
      Serial.print(portAString);
      Serial.print(" portB:");
      Serial.println(portBString);
      lastPortA = portA;
      lastPortB = portB;
    }
  }
}

void actionTrainDetected(int sensorID)
{
  ledRed(sensorID);
  occupiedSection(sensorID);
}

void trackClear(int sensorID)
{
  int previousSensor = sensorID;
  bool searching = true;
  while (searching == true)
  {
    // Serial.print("Searching - ");
    previousSensor--;
    if (previousSensor < 0)
    {
      // Serial.print(" Wrapped to sensor 15");
      previousSensor = 15;
    }
    if (sensorActive[previousSensor] == true)
    {
      searching = false;
    }
    else
    {
      // Serial.print("sensor inactive ");
    }
  }
  // Serial.print("Clearing Track:");
  // Serial.println(previousSensor);
  if (manualState[previousSensor] == false && stopPressed == false)
  {
    // manual switch is not active so can change to Green
    ledGreen(previousSensor);
  }
  clearSection(previousSensor); // do clear the train from the previous section
}

// actions
void trainDetected(int sensorID)
{
  Serial.print("Train Detected Sensor:");
  Serial.println(sensorID);
  actionTrainDetected(sensorID);
}

void trackCleared(int sensorID)
{
  Serial.print("Track Cleared Sensor:");
  Serial.println(sensorID);
  trackClear(sensorID);
}

void manualStopPressed(int sensorID)
{
  // Serial.print("Sensor:");
  // Serial.print(sensorID);
  // Serial.println(" Manual Stop Activated");
  ledRed(sensorID); // indicate that the stop is on
  manualState[sensorID] = true;
}

void manualStopReleased(int sensorID)
{
  manualState[sensorID] = false;
  // check if we can clear the red led on the board
  // Serial.print("Sensor:");
  // Serial.print(sensorID);
  // Serial.println(" Manual Stop Released");
  uint16_t check = 0x0001 << sensorID;
  // Serial.print("check:");
  // Serial.print(check);
  // Serial.print(" portB:");
  // Serial.println(portB);
  if ((check & portB)) // portB pin low if train in section
  {
    // there is not a train in the section
    if (stopPressed == false)
    {
      // stop button not pressed so we can go back to green
      // Serial.println("Track clear so going Green");
      ledGreen(sensorID);
    }
  }
}

void forceTrackClear(int sensorID)
{
  // force clear of this sensors section of track (used for steaming bays)
  uint16_t check = 0x0001 << sensorID;
  portB = portB | check;
}

unsigned char checksum(char *ptr, size_t sz)
{
  unsigned char chk = 0;
  while (sz-- != 0)
    chk -= *ptr++;
  return chk;
}

void checkMessage()
{
  static uint16_t lastPortA = 0;
  static uint16_t lastPortB = 0;
  char *message = getUdpMessage();
  int messageLen = strlen(message);
  if (messageLen > 5)
  {
    //    Serial.print("checkSMessage->message: ");
    //    Serial.println(message);
    //    Serial.print("checkMessage->messageLen: ");
    //    Serial.println(messageLen);
    if (isMaster)
    {
      // this is a master so process the messages from sensors
      if (compareBuffers(message, "Sensor", 6))
      {
        // message from Sensor so update it's IP address in case it has changed
        int sensorID = message[7] - 48; // convert from ASCII
        // Serial.print("SensorID:");
        // Serial.println(sensorID);
        if (sensorID >= 0 && sensorID <= 15)
        {
          if (sensorActive[sensorID] == false)
          {
            Serial.printf("Aquired Sensor:%d running software %s\n", sensorID, &message[8]);
          }
          // sensorAddress[sensorID] = lastIP;
          sensorActive[sensorID] = true;
          seenActive[sensorID] = true; // indicate seen in this cycle
          // Serial.print("Seen Sensor:");
          // Serial.println(sensorID);
          digitalWrite(signalsActive, LOW); // turn on the signals active LED
        }

        // check for specific messages
        if (compareBuffersFrom(message, "Train Detected", 9, 13))
        {
          trainDetected(sensorID);
        }
        else if (compareBuffersFrom(message, "Track Clear", 9, 11))
        {
          trackCleared(sensorID);
        }
        else if (compareBuffersFrom(message, "Manual Stop Active", 9, 18))
        {
          manualStopPressed(sensorID);
        }
        else if (compareBuffersFrom(message, "Manual Stop Released", 9, 20))
        {
          manualStopReleased(sensorID);
        }
        else if (compareBuffersFrom(message, "Force Track Clear", 9, 17))
        {
          forceTrackClear(sensorID);
        }
        else
        {
          // Serial.println("Track occupied so staying Red");
        }
      }
      else if (compareBuffers(message, "Contrl", 6))
      {
        // It is a control message sent from the WHRApplication
        // config message format
        //        Imsc
        // Contrl:DDDs
        // 0000000000111111111122222222223
        // 0123456789012345678901234567890
        // ID = Sensor ID
        // mD = minDistance 1...250 (distance is + 50 cm)
        // sD = spanDistance 1...250 in cm
        // cs = checksum
        //
        // sensorStart and sensorSpan are the cm values for those attributes
        //        Serial.printf("Received message from WHRDisplay App %s\n", message);
        int sensorID = message[7] - 48; // convert from ASCII
        int ctrStart = message[8];
        int ctrSpan = message[9];
        Serial.printf("Received message from WHRDisplay App SensorID:%d minDistance:%d spanDistance:%d checkSum:%d\n", sensorID, ctrStart, ctrSpan, message[10]);
        //        int ctrCS = message[14];
        //        if (checksum(message, 10) == 0)
        //        {
        Serial.println("Checksum OK");
        // Serial.print("SensorID:");
        // Serial.println(sensorID);
        if (sensorID >= 0 && sensorID <= 15)
        { // a valid sensorID
          if (ctrStart != 255)
          {
            sensorStart[sensorID] = ctrStart;
            Serial.printf("new minDistance of %d for Sensor %d\n", ctrStart, sensorID);
          }
          if (ctrSpan != 255)
          {
            sensorSpan[sensorID] = ctrSpan;
            Serial.printf("new spanDistance of %d for Sensor %d\n", ctrSpan, sensorID);
          }
        }
        else
        {
          // check if it is all clear or all stop or all go
          if (sensorID == 35)
          {
            // All Stop
            portA = 0xffff;
          }
          if (sensorID == 40)
          {
            // All Clear
            portB = 0xffff;
          }
          if (sensorID == 23)
          {
            // all Go
            portA = 0;
          }
        }
        //       }
      }
    }
    else
    {
      // in slave mode so look for message from signal box
      if (compareBuffers(message, "Signalbox Address:", 18))
      {
        // process the light status
        uint8_t byte1 = message[18];
        uint8_t byte2 = message[19];
        uint16_t signalStates = byte1 << 8;
        signalStates = signalStates | byte2;
        byte1 = message[20];
        byte2 = message[21];
        uint16_t trackStates = byte1 << 8;
        trackStates = trackStates | byte2;
        portA = signalStates;
        portB = trackStates;
        writePortA(portA);
        writePortB(portB);
        if (lastPortA != portA || lastPortB != portB)
        {
          Serial.print("portA:");
          Serial.print(binaryString(portA));
          Serial.print("  portB:");
          Serial.println(binaryString(portB));
          lastPortA = portA;
          lastPortB = portB;
        }
      }
    }
  }
}

void checkMaster()
{
  if (digitalRead(master) == isMaster)
  {
    isMaster = !digitalRead(master); // switch is low for master High for Slave
    if (isMaster)
    {
      Serial.println("Configured as Master");
    }
    else
    {
      Serial.println("Configured as Slave");
    }
  }
}

void checkSwitches()
{
  checkMaster();
  if (isMaster)
  {
    if (digitalRead(clearButton) == false)
    {
      if (clearPressed == false)
      {
        // Serial.println("Clear button Pressed");
        if (stopPressed == false)
        {
          clearPressed = true;
          // clear all trains from board if the stop button is not pressed
          for (int i = 0; i < 16; i++)
          {
            clearSection(i);
            ledGreen(i);
          }
        }
      }
    }
    else
    {
      clearPressed = false;
    }
    if (digitalRead(stopButton) == false)
    {
      // stop button is pressed
      if (stopPressed == false)
      {
        // it wasn't pressed before
        // Serial.println("Stop button pressed");
        stopPressed = true;
        for (int i = 0; i < 16; i++)
        {
          // turn all the lights to red
          ledRed(i);
        }
      }
    }
    else
    {
      // stop button not pressed
      if (stopPressed == true)
      {
        // button just released
        stopPressed = false;
        for (int i = 0; i < 16; i++)
        {
          uint16_t check = 0x0001 << i;
          // Serial.print("check:");
          // Serial.print(check);
          // Serial.print(" portB:");
          // Serial.println(portB);
          if ((check & portB)) // portB pin low if train in section
          {
            // there is not a train in this section
            // stop button not pressed so we can go back to green
            // Serial.println("Track clear so going Green");
            ledGreen(i);
          }
        }
      }
    }
  }
}

void demoFlash(int pulses)
{
  static bool ledState = HIGH;
  for (int j = 0; j < pulses; j++)
  {
    ledState = !ledState;
    if (ledState == false)
    {
      digitalWrite(signalsActive, HIGH);
    }
    else
    {
      digitalWrite(signalsActive, LOW);
    }
    delay(500);
  }
}

void demoMode(void)
{
  // Demo mode
  stopPressed = false;
  if (!digitalRead(clearButton) && !digitalRead(stopButton))
  {
    // hold down clear and stop buttons to enter test mode
    int i = 0;
    Serial.println("Demo Mode");
    sensorActive[i] = true;
    writePortA(portA);
    writePortB(portB);
    sendAddressMessage();
    while (!digitalRead(stopButton)) // demo will run as long as stop is pressed
    {
      // make a train travel around the track
      actionTrainDetected(i); // simulate having received a train detected message from sensor i
      sendAddressMessage();
      demoFlash(3);
      if (!digitalRead(clearButton))
      {
        // hold down the clear button until the final section of track shows occupied
        sensorActive[i + 1] = true;
      }
      trackClear(i); // simulate having received a train clear messagefrom sensor i
      sendAddressMessage();
      demoFlash(10);
      i++;
      if (sensorActive[i] == false)
      {
        i = 0;
      }
    }

    for (i = 0; i < 16; i++)
    {
      // clear the demo mode sensors active flags
      sensorActive[i] = false;
    }
    portA = 0x0000;
    portB = 0XFFFF;
    writePortA(portA);
    writePortB(portB);
  }
}

void initUART()
{
  // setup the i2C UART
  portA = 0x0000;
  portB = 0XFFFF;

  digitalWrite(uartReset, LOW);
  delay(100);
  digitalWrite(uartReset, HIGH);
  delay(100);

  Wire.begin();
  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA
  Wire.write(0x00); // All output
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x01); // IODIRB
  Wire.write(0x00); // All output
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x00); // IODIRA
  Wire.write(0x00); // All output
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x01); // IODIRB
  Wire.write(0x00); // All output
  Wire.endTransmission();

  writePortA(0xFFFF); // all red LED on
  writePortB(0xFFFF); // all yellow LED off
}

void checkEEPROM()
{
#define EEPROMtestValue 0x5A
  // check for EEPROM
  uint8_t tries = 0;
  while (tries < 5 && eepromAvailable == false)
  {
    tries++;
    uint8_t value = eeprom.read(EEPROMtestaddr);
    Serial.print("EEPROM first read value:");
    Serial.println(value);

    if (value == EEPROMtestValue)
    {
      // EEPROM exists
      eepromAvailable = true;
      Serial.print("EEPROM exists on try ");
      Serial.println(tries);
      // read the values from the EEPROM
      for (int i = 0; i <= 15; i++)
      {
        delay(10);
        sensorStart[i] = eeprom.read(EEPROMstartaddr + i);
        delay(10);
        sensorSpan[i] = eeprom.read(EEPROMspanaddr + i);
      }
      Serial.println("Settings from EEPROM");
      Serial.println("Sensor   Start   Span  (from    to   )");
      for (int i = 0; i <= 15; i++)
      {
        char buf[50];
        sprintf(buf, "  %02d     %03d     %03d   (%0.2fm   %0.2fm)", i, sensorStart[i], sensorSpan[i], float(sensorStart[i] + 50) / 100, float(sensorStart[i] + 50) / 100 + float(sensorSpan[i]) / 100);
        Serial.println(buf);
      }
    }
    else
    {
      // try writing as it might be a new eeprom
      eeprom.write(EEPROMtestaddr, EEPROMtestValue);
      delay(100);
      value = eeprom.read(EEPROMtestaddr);
      Serial.print("EEPROM second read value:");
      Serial.println(value);
      if (value == EEPROMtestValue)
      {
        // EEPROM exists
        eepromAvailable = true;
        Serial.println("EEPROM exists first use");
        // set the defaults in teh EEPROM as it is first use
        for (int i = 0; i <= 15; i++)
        {
          delay(10);
          eeprom.write(EEPROMstartaddr + i, sensorStart[i]);
          delay(10);
          eeprom.write(EEPROMspanaddr + i, sensorSpan[i]);
        }
        Serial.println("Set to EEPROM");
        Serial.println("Sensor   Start   Span");
        for (int i = 0; i <= 15; i++)
        {
          char buf[50];
          sprintf(buf, "  %02d     %03d     %03d   (%0.2fm   %0.2fm)", i, sensorStart[i], sensorSpan[i], float(sensorStart[i] + 50) / 100, float(sensorStart[i] + 50) / 100 + float(sensorSpan[i]) / 100);
          Serial.println(buf);
        }
      }
      else
      {
        Serial.println("No EEPROM fitted");
      }
    }
    delay(500);
  }
}

void saveSettings()
{
  if (eepromAvailable == true)
  {
    uint8_t value;
    for (int i = 0; i <= 15; i++)
    {
      value = eeprom.read(EEPROMstartaddr + i);
      if (value != sensorStart[i])
      {
        Serial.print("Writing new start value:");
        Serial.print(sensorStart[i]);
        Serial.print(" for sensor ");
        Serial.println(i);
        eeprom.write(EEPROMstartaddr + i, sensorStart[i]);
      }
      value = eeprom.read(EEPROMspanaddr + i);
      if (value != sensorSpan[i])
      {
        Serial.print("Writing new span value:");
        Serial.print(sensorSpan[i]);
        Serial.print(" for sensor ");
        Serial.println("i");
        eeprom.write(EEPROMspanaddr + i, sensorSpan[i]);
      }
    }
  }
}

void setup()
{
  Serial.begin(115200); // // Serial Communication is starting with 115200 baudrate
  delay(2000);
  Serial.println("");
  Serial.println("West Huntspill Railway Signalbox Module"); // print some text in Serial Monitor
  Serial.printf("Software Version: %.2f\n", softwareVersion);

  pinMode(wifiActive, OUTPUT);
  pinMode(signalsActive, OUTPUT);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(clearButton, INPUT_PULLUP);
  pinMode(master, INPUT_PULLUP); // master slave input switch
  pinMode(uartReset, OUTPUT);    // UART reset

  digitalWrite(wifiActive, HIGH);    // turn off the wifi active LED
  digitalWrite(signalsActive, HIGH); // turn off the signals active LED

  isMaster = digitalRead(master); // force print of status on first check

  checkMaster();

  initUART();

  setupWiFi();

  udp.begin(udpPort);

  checkEEPROM();

  demoMode(); // check if entering demo mode

  writePortA(portA); // all green LED on
  writePortB(portB); // all yellow LED off

  sendAddress = false;
  clearSensorActive = false;
  fastTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(fastTimer, &timerInterrupt, true);
  timerAlarmWrite(fastTimer, 500000, true);
  timerAlarmEnable(fastTimer);

  slowTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(slowTimer, &slowTimerInterrupt, true);
  timerAlarmWrite(slowTimer, 20000000, true);
  timerAlarmEnable(slowTimer);

  stopPressed = !digitalRead(stopButton);
  for (int i = 0; i < 16; i++)
  {
    if (stopPressed == true)
    {
      ledRed(i);
    }
  }
}



void loop()
{

  checkMaster();
  checkMessage();
  checkSwitches();

  if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(wifiActive, LOW);    // turn on the wifi active LED
  } else {
      digitalWrite(wifiActive, HIGH);    // turn off the wifi active LED
  }

  if (sendAddress == true) // set true by teh fast timer interrupt every 0.5 seconds
  {
    sendAddressMessage();
  }
  if (clearSensorActive == true) // set true by the slow timer interrupt every 20 seconds
  {
    saveSettings();
    for (int i = 0; i < 16; i++)
    {
      if (seenActive[i] == false && sensorActive[i] == true)
      {
        // didn't see it last time
        sensorActive[i] = false; // show this sensor has is no longer active
        Serial.print("Lost Sensor:");
        Serial.println(i);
      }
      seenActive[i] = false; // clear the sensors (they will repopulate on next receipt of message)
    }
    clearSensorActive = false;
  }
}
