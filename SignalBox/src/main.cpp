#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>

//const char *ssid = "WHR Signals";
const char *ssid = "Barnard Home Network";
const char *pwd = "0D03908CE5"; // set for suitable password
const int udpPort = 44444;
WiFiUDP udp;
uint8_t txBuffer[50] = "";
uint8_t rxBuffer[50] = "";
IPAddress lastIP;

//IPAddress sensorAddress[16];

bool sensorActive[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
bool seenActive[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

hw_timer_t *fastTimer = NULL;
hw_timer_t *slowTimer = NULL;

// defines variables
bool ledState;
bool detected;
bool sendAddress;
bool clearSensorActive;
bool clearPressed = false;
bool stopPressed = false;
bool isMaster;

uint16_t portA;
uint16_t portB;

bool manualState[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; // indicates manual stop switch is active

int demoSteps = 7; // the default number of lights on the board for demo mode

#define wifiActive 4
#define signalsActive 2
#define stopButton 15
#define clearButton 16
#define master 34
#define uartReset 17

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
  //Serial.print("bit pattern:");
  //Serial.print(value, BIN);
  //Serial.print(" for sensor:");
  //Serial.println(sensor);
  Serial.print("RED on for sensor:");
  Serial.println(sensor);
  portA = portA | value;
  writePortA(portA);
}

void ledGreen(int sensor)
{
  uint16_t value = 0x0001 << sensor;
  value = value ^ 0xFFFF;
  //Serial.print("bit pattern:");
  //Serial.print(value, BIN);
  //Serial.print(" for sensor:");
  //Serial.println(sensor);
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
  //Serial.print("bit pattern:");
  //Serial.print(value, BIN);
  //Serial.print(" for sensor:");
  //Serial.println(sensor);
  portB = portB & value;
  writePortB(portB);
  Serial.print("YELLOW on for track:");
  Serial.println(sensor);
}

void clearSection(int sensor)
{
  // turn off the yellow led to show the section is clear
  uint16_t value = 0x0001 << sensor;
  //Serial.print("bit pattern:");
  //Serial.print(value, BIN);
  //Serial.print(" for sensor:");
  //Serial.println(sensor);
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
    //Serial.print("buf1:");
    //Serial.print(buf1[i]);
    //Serial.print("  buf2:");
    //Serial.println(buf2[i]);
  }
  return true;
}

bool compareBuffersFrom(char *buf1, const char *buf2, int from, int num)
{
  if (num == 0)
    return false;

  //Serial.print("from:");
  //Serial.print(from);
  //Serial.print("  num:");
  //Serial.println(num);

  for (int i = from; i < (num + from); i++)
  {
    //Serial.print("buf1:");
    //Serial.print(buf1[i]);
    //Serial.print("  buf2:");
    //Serial.println(buf2[i - from]);
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
    //Serial.print("getUdpMessage->Received: ");
    //Serial.println((char *)buffer);
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
  static uint16_t lastPortA = 0;
  static uint16_t lastPortB = 0;
  if (isMaster)
  {
    //  This bradcasts the signalbox IP address if the signalbox is a master
    copyToTxBuffer("Signalbox Address: ");
    uint8_t aByte = portA & 0xff;
    txBuffer[19] = aByte; // put in the bit p[attern for the lights]
    aByte = portA >> 8;
    txBuffer[18] = aByte; // put in the bit p[attern for the lights]
    aByte = portB & 0xFF;
    txBuffer[21] = aByte;
    aByte = portB >> 8;
    txBuffer[20] = aByte; // put in the bit p[attern for the lights]

    const char *adr = WiFi.localIP().toString().c_str();
    int adrLen = strlen(adr);
    //Serial.println(adrLen);
    memcpy(&txBuffer[22], adr, adrLen);
    //Serial.println((char *)buffer);
    udp.beginPacket("255.255.255.255", udpPort);
    udp.write(txBuffer, adrLen + 22);
    //udp.write(buffer,33);
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

void trainDetected(int sensorID)
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
    //Serial.print("Searching - ");
    previousSensor--;
    if (previousSensor < 0)
    {
      //Serial.print(" Wrapped to sensor 15");
      previousSensor = 15;
    }
    if (sensorActive[previousSensor] == true)
    {
      searching = false;
    }
    else
    {
      //Serial.print("sensor inactive ");
    }
  }
  //Serial.print("Clearing Track:");
  //Serial.println(previousSensor);
  if (manualState[previousSensor] == false && stopPressed == false)
  {
    // manual switch is not active so can change to Green
    ledGreen(previousSensor);
  }
  clearSection(previousSensor); // do clear the train from the previous section
}

void checkMessage()
{
  static uint16_t lastPortA = 0;
  static uint16_t lastPortB = 0;
  char *message = getUdpMessage();
  int messageLen = strlen(message);
  if (messageLen > 5)
  {
    //Serial.print("checkSMessage->message: ");
    //Serial.println(message);
    //Serial.print("checkMessage->messageLen: ");
    //Serial.println(messageLen);
    if (isMaster)
    {
      // this is a master so process the messages from sensors
      if (compareBuffers(message, "Sensor", 6))
      {
        // message from Sensor so update it's IP address in case it has changed
        int sensorID = message[7] - 48; // convert from ASCII
        //Serial.print("SensorID:");
        //Serial.println(sensorID);
        if (sensorID >= 0 && sensorID <= 15)
        {
          if (sensorActive[sensorID] == false)
          {
            Serial.print("Aquired Sensor:");
            Serial.println(sensorID);
          }
          //sensorAddress[sensorID] = lastIP;
          sensorActive[sensorID] = true;
          seenActive[sensorID] = true; // indicate seen in this cycle
          //Serial.print("Seen Sensor:");
          //Serial.println(sensorID);
          digitalWrite(signalsActive, LOW); // turn on the signals active LED
        }

        // check for specific messages
        if (compareBuffersFrom(message, "Train Detected", 9, 13))
        {
          Serial.print("Train Detected Sensor:");
          Serial.println(sensorID);
          trainDetected(sensorID);
        }
        else if (compareBuffersFrom(message, "Track Clear", 9, 11))
        {
          Serial.print("Track Cleared Sensor:");
          Serial.println(sensorID);
          trackClear(sensorID);
        }
        else if (compareBuffersFrom(message, "Manual Stop Active", 9, 18))
        {
          //Serial.print("Sensor:");
          //Serial.print(sensorID);
          //Serial.println(" Manual Stop Activated");
          ledRed(sensorID); // indicate that the stop is on
          manualState[sensorID] = true;
        }
        else if (compareBuffersFrom(message, "Manual Stop Released", 9, 20))
        {
          manualState[sensorID] = false;
          // check if we can clear the red led on the board
          //Serial.print("Sensor:");
          //Serial.print(sensorID);
          //Serial.println(" Manual Stop Released");
          uint16_t check = 0x0001 << sensorID;
          //Serial.print("check:");
          //Serial.print(check);
          //Serial.print(" portB:");
          //Serial.println(portB);
          if ((check & portB)) // portB pin low if train in section
          {
            // there is not a train in the section
            if (stopPressed == false)
            {
              // stop button not pressed so we can go back to green
              //Serial.println("Track clear so going Green");
              ledGreen(sensorID);
            }
          }
        }
        else if (compareBuffersFrom(message, "Force Track Clear", 9, 17)){
          // force clear of this sensors section of track (used for steaming bays)
          uint16_t check = 0x0001 << sensorID;
          portB = portB | check;
        } else 
        {
          //Serial.println("Track occupied so staying Red");
        }
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
        if(lastPortA != portA || lastPortB != portB){
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
        //Serial.println("Clear button Pressed");
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
        //Serial.println("Stop button pressed");
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
          //Serial.print("check:");
          //Serial.print(check);
          //Serial.print(" portB:");
          //Serial.println(portB);
          if ((check & portB)) // portB pin low if train in section
          {
            // there is not a train in this section
            // stop button not pressed so we can go back to green
            //Serial.println("Track clear so going Green");
            ledGreen(i);
          }
        }
      }
    }
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
      trainDetected(i); // simulate having received a train detected message from sensor i
      sendAddressMessage();
      delay(1500);
      if (!digitalRead(clearButton))
      {
        // hold down the clear button until the final section of track shows occupied
        sensorActive[i + 1] = true;
      }
      trackClear(i); // simulate having received a train clear messagefrom sensor i
      sendAddressMessage();
      delay(5000);
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

void setup()
{
  Serial.begin(115200); // // Serial Communication is starting with 115200 baudrate
  delay(2000);
  Serial.println("");
  Serial.println("West Huntspill Railway Signalbox Module"); // print some text in Serial Monitor

  pinMode(wifiActive, OUTPUT);
  pinMode(signalsActive, OUTPUT);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(clearButton, INPUT_PULLUP);
  pinMode(master, INPUT_PULLUP); // master slave input switch
  pinMode(uartReset, OUTPUT);    // UART reset

  digitalWrite(wifiActive, HIGH);    // turn off the wifi active LED
  digitalWrite(signalsActive, HIGH); // turn off the signals active LED
  isMaster = digitalRead(master);    // force print of status on first check
  checkMaster();

  portA = 0x0000;
  portB = 0XFFFF;
  digitalWrite(uartReset, LOW);
  delay(100);
  digitalWrite(uartReset, HIGH);
  delay(100);
  // setup the i2C UART
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

  detected = false;

  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to:");
  Serial.print(ssid);
  Serial.print(" IP Address:");
  Serial.println(WiFi.localIP());

  digitalWrite(wifiActive, LOW); // turn on the wifi led
  delay(1000);
  udp.begin(udpPort);

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

  if (sendAddress == true)
  {
    sendAddressMessage();
  }
  if (clearSensorActive == true)
  {
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
