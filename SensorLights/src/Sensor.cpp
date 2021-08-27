#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>

//const char *ssid = "WHR Signals"; // WiFi Network SSID
const char *ssid = "Barnard Home Network"; // WiFi Network SSID
const char *pwd = "0D03908CE5";            // WiFi network password
int minDistance = 30;                      // Minimum distance to trigger sensor in cm
int maxDistance = 60;                      // Maximum distance to trigger sensor in cm
#define debounceThreshold 40               // Howmany times to see the sensor active before operating
#define retriggerThreshold 100             //  How many times to see the sensor inactive to enable re trigger
#define messageRepeat 3                    // define how many times to repreat a message

const char *signalboxAddress = "255.255.255.255"; // ensure the storage is large enough
const int udpPort = 44444;
const char *payload[20];
WiFiUDP udp;
uint8_t txBuffer[150] = "";
uint8_t rxBuffer[150] = "";

#define echoPin 27     // Echo of HC-SR04
#define trigPin 26     // Trig of HC-SR04
#define sensedPin 18   // LED to show when sensor has detected
#define activePin 17   // LED to show when there is an active signalbox
#define wifiPin 16     // LED to indicate connected to WiFi
#define output1Pin 5   // output 1 (MOSFET high current)
#define output2Pin 4   // output 2 (MOSFET high current)
#define manualStop 15  // input for manual stop button (used at station)
#define sensorStart 36 // ADC input for sensor start distance
#define sensorSpan 39  // ADCinput for sensor span distance

#define localStop 14   // high if using local stop button
#define localSense 13  // high if using local sensor
#define joinWiFiHub 25 // high is connecting to wifi network (low then create the network)

#define addb0 32 // address lines to set sensorID
#define addb1 33
#define addb2 34
#define addb3 35

// defines variables
long duration; // variable for the duration of sound wave travel
int distance;  // variable for the distance measurement
uint8_t sensorID;
bool ledState;
bool detected;
int debounce;
long retrigger;
bool haveSignalboxAddress;
bool manualStopActive = false;
bool autoStopActive = false;
bool myTrack = false;

hw_timer_t *timer = NULL;
bool sendAlive = false;
bool flashing = false;

void timerInterrupt()
{
  sendAlive = true;
}

uint8_t getSensorID()
{
  uint8_t iD = 0;
  if (digitalRead(addb0) == true)
  {
    iD = iD + 1;
  }
  if (digitalRead(addb1) == true)
  {
    iD = iD + 2;
  }
  if (digitalRead(addb2) == true)
  {
    iD = iD + 4;
  }
  if (digitalRead(addb3) == true)
  {
    iD = iD + 8;
  }
  return iD;
}

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);  // Sets the echoPin as an INPUT
  pinMode(sensedPin, OUTPUT);
  pinMode(activePin, OUTPUT);
  pinMode(wifiPin, OUTPUT);
  pinMode(output1Pin, OUTPUT);
  pinMode(output2Pin, OUTPUT);
  pinMode(manualStop, INPUT_PULLUP);
  pinMode(addb0, INPUT_PULLUP);
  pinMode(addb1, INPUT_PULLUP);
  pinMode(addb2, INPUT_PULLUP);
  pinMode(addb3, INPUT_PULLUP);
  pinMode(localStop, INPUT_PULLUP);
  pinMode(localSense, INPUT_PULLUP);
  pinMode(joinWiFiHub, INPUT_PULLUP);
  pinMode(sensorStart, INPUT);
  pinMode(sensorSpan, INPUT);
  digitalWrite(sensedPin, HIGH);
  digitalWrite(activePin, HIGH);
  digitalWrite(wifiPin, HIGH);
  digitalWrite(output1Pin, LOW);
  digitalWrite(output2Pin, LOW);
  delay(1000);
  // read device address
  sensorID = getSensorID();

  delay(2000);          // let the debugger switch windows
  Serial.begin(115200); // // Serial Communication is starting with 115200 of baudrate speed
  Serial.println("");
  if (digitalRead(joinWiFiHub) == true)
  {
    Serial.print("West Huntspill Railway Sensor Module activated with ID:");
  }
  else
  {
    Serial.print("West Huntspill Railway Sensor Module and WiFi Hub activated with ID:");
  }
  Serial.println(sensorID);
  if (digitalRead(localStop))
  {
    Serial.println("Supports a local stop button");
  }
  else
  {
    Serial.println("Local Stop disabled");
  }
  if (digitalRead(localSense))
  {
    Serial.println("Supports a local train sensor");
  }
  else
  {
    Serial.println("Train sensor disabled");
  }

  debounce = 0;
  detected = false;
  retrigger = 0;
  if (digitalRead(joinWiFiHub) == true)
  { // join the wifi network
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
  }
  else
  {
    // create the wifi network
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, pwd);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  }
  digitalWrite(wifiPin, LOW); // indicate we are connected to WiFi
  haveSignalboxAddress = false;

  udp.begin(udpPort);
  Serial.println("Waiting for Signalbox Address");
  flashing = false;

  sendAlive = false;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerInterrupt, true);
  timerAlarmWrite(timer, 10000000, true);
  timerAlarmEnable(timer);
  sensorID = getSensorID();
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

char *getUdpMessage()
{
  udp.parsePacket();
  rxBuffer[0] = 0;
  if (udp.read(rxBuffer, 50) > 0)
  {
    // received a message
    //Serial.print("getUdpMessage->Received: ");
    //Serial.println((char *)buffer);
  }
  return (char *)rxBuffer;
}

void copyToTxBuffer(const char *str)
{
  memcpy(txBuffer, str, strlen(str) + 1);
}

void copyToBuffer(char *theBuffer, const char *str)
{
  memcpy(theBuffer, str, strlen(str) + 1);
}

void checkSignalboxMessage()
{
  static uint16_t lastLight = 0xFFFF;
  char *message = getUdpMessage();
  int messageLen = strlen(message);
  //Serial.print("checkSignalboxMessage->messageLen: ");
  //Serial.println(messageLen);
  if (messageLen > 1)
  {
    //Serial.print("checkSignalboxMessage->message: ");
    //Serial.println(message);
    if (compareBuffers(message, "Signalbox Address:", 18))
    {
      // Signalbox address message (this is broadcast so all sensors get it)
      memcpy(&payload, &message[22], 16);
      //Serial.print("checkSignalboxMessage->payload: ");
      //Serial.println((const char *)payload);
      if (compareBuffers((char *)signalboxAddress, (const char *)payload, strlen(signalboxAddress)) == false)
      { // the address received is different to the one we had
        signalboxAddress = (const char *)payload;
        //memcpy(&signalboxAddress, payload, 20);
        Serial.println("");
        Serial.print("Received new signalbox address:");
        Serial.println(signalboxAddress);
        haveSignalboxAddress = true;
        digitalWrite(activePin, LOW); // turn on the LED to show we have active Signalbox
        //digitalWrite(output2Pin, HIGH); // turn on the Green light once we have established a link
      }
      else
      {
        //Serial.println("checkSignalboxMessage->signalboxAddress == payload");
        haveSignalboxAddress = true;
        digitalWrite(activePin, LOW); // turn on the LED to show we have active Signalbox
      }

      // process the light status
      uint8_t byte1 = message[18];
      uint8_t byte2 = message[19];
      uint16_t signalStates = byte1 << 8;
      signalStates = signalStates | byte2;
      byte1 = message[20];
      byte2 = message[21];
      uint16_t trackStates = byte1 << 8;
      trackStates = trackStates | byte2;
      // set the signal to the signalbox value
      //Serial.print("signalStates from signalbox:");
      //Serial.println(binaryString(signalStates));
      uint16_t myLight = signalStates >> sensorID;
      myLight = myLight & 0x0001;
      uint16_t theTrack = trackStates >> sensorID;
      myTrack = theTrack & 0x0001;
      if (flashing == false)
      {
        if (myLight != 0)
        {
          if (lastLight != myLight)
          {
            Serial.println("myLight: RED");
            lastLight = myLight;
          }
          digitalWrite(output1Pin, HIGH); // red on
          digitalWrite(output2Pin, LOW);  // green off
          autoStopActive = true;
        }
        else
        {
          if (lastLight != myLight)
          {
            Serial.println("myLight: GREEN");
            lastLight = myLight;
          }
          digitalWrite(output1Pin, LOW);  // red off
          digitalWrite(output2Pin, HIGH); // green on
          autoStopActive = false;
          manualStopActive = false; // force resent of manual stop if the signalbox clears
        }
      }
    }
  }
}

void sendMessageToSignalbox(const char *mess)
{
  int lenMess = strlen(mess);
  copyToTxBuffer(mess);
  txBuffer[7] = sensorID + 48; // make it ASCII for send
  for (int i = 1; i <= messageRepeat; i++)
  {
    udp.beginPacket("255.255.255.255", udpPort);
    udp.write(txBuffer, lenMess + 1);
    udp.endPacket();
    //Serial.print("Sent:");
    //Serial.println(mess);
  }
}

void checkManualStop()
{
  bool currentState = !digitalRead(manualStop);
  if (manualStopActive == false && currentState == true)
  {
    // button pressed and it wasn't pressed before
    manualStopActive = true;
    sendMessageToSignalbox("Sensor:  Manual Stop Active");
    Serial.println("Manual Stop button pressed");
  }
  else if (manualStopActive == true && currentState == false)
  {
    // button just released
    manualStopActive = false;
    sendMessageToSignalbox("Sensor:  Manual Stop Released");
    Serial.println("Manual Stop button released");
    if (autoStopActive == false)
    {
      // automatic light state would have been green
      //digitalWrite(output1Pin, LOW);  // red off
      //digitalWrite(output2Pin, HIGH); // green on
    }
    else
    {
      if (myTrack == false)
      {
        if (digitalRead(localStop))
        { // This SensorLights module handles a local stop button in normal mode
          // my track is occupied so flash the LED
          flashing = true; // stop the auto light setting
          for (int i = 0; i < 3; i++)
          {
            digitalWrite(output1Pin, LOW); // red off
            delay(500);
            digitalWrite(output1Pin, HIGH); // red on
            delay(500);
            digitalWrite(output1Pin, LOW); // red off
            delay(500);
            digitalWrite(output1Pin, HIGH); // red on
            delay(500);
          }
        }
        else
        {
          // This SensorLights module handles local stop with forced clear on release
          Serial.println("Force Track clear");
          sendMessageToSignalbox("Sensor:  Force Clear");
        }
      }
      flashing = false;
    }
  }
}

void sendAliveMessage()
{
  copyToTxBuffer("Sensor:                ");
  txBuffer[7] = sensorID + 48;
  udp.beginPacket("255.255.255.255", udpPort);
  udp.write(txBuffer, 20);
  udp.endPacket();
  //Serial.println("Sensor Alive message sent");
  sendAlive = false;
}

void checkSensor(void)
{
  if (digitalRead(localSense))
  {
    //Serial.println("Reading distance sensor");
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (out and back)
    if ((distance >= minDistance) && (distance <= maxDistance))
    {
      // detected object
      //Serial.print("measured distance:");
      //Serial.print(distance);
      //Serial.print("cm - debounce:");
      //Serial.println(debounce);
      debounce = debounce + 1;
      if (debounce >= debounceThreshold)
      {
        // Seen the object long enough that it is real
        if (detected == false)
        {
          // first time we have seen it
          digitalWrite(sensedPin, LOW); // turn on sensed LED
          Serial.print("Train Detected at ");
          Serial.print(distance);
          Serial.println(" cm");
          sendMessageToSignalbox("Sensor:  Train Detected");

          detected = true;
        }
        debounce = 0;
        retrigger = 0;
      }
    }
    else
    {
      // no in range object detected
      debounce = 0;
      if (detected == true)
      {
        // we previously had seen an object so action the retrigger delay
        retrigger = retrigger + 1;
        if (retrigger >= retriggerThreshold)
        {
          // retrigger delay expired enable for re sensing objects
          digitalWrite(sensedPin, HIGH); // turn off sensed LED
          Serial.println("Track Clear");
          sendMessageToSignalbox("Sensor:  Track Clear");
          detected = false;
          retrigger = 0;
        }
      }
    }
  }
}

void setDistance(void)
{
  static int lastStart = 0;
  static int lastSpan = 0;
  static int lastMinDistance = 0;
  static int lastMaxDistance = 0;
  int startReading = analogRead(sensorStart);
  int spanReading = analogRead(sensorSpan);
  startReading = startReading >> 4; // remove noise
  spanReading = spanReading >> 4;   // remove noise
  if (lastStart != startReading || lastSpan != spanReading)
  {
    lastStart = startReading;
    lastSpan = spanReading;
    // sensor measures 0 to 4m
    minDistance = (startReading) + 50;
    maxDistance = minDistance + (spanReading) + 10;
    //Serial.print("startReading:");
    //Serial.print(startReading);
    //Serial.print("   spanReading:");
    //Serial.println(spanReading);
    if (abs(minDistance - lastMinDistance) > 10 || abs(maxDistance - lastMaxDistance) > 10)
    {
      //Serial.print("minDistance:");
      //Serial.print(minDistance);
      //Serial.print("cm  maxDistance:");
      //Serial.print(maxDistance);
      //Serial.println("cm");
      lastMinDistance = minDistance;
      lastMaxDistance = maxDistance;
      char theBuffer[100];
      char theNumber[5];
      copyToBuffer(&theBuffer[0], "Sensor:  Range Adjustment minDistance:     ");
      sprintf(theNumber, "%4d", minDistance);
      copyToBuffer(&theBuffer[38], theNumber);
      copyToBuffer(&theBuffer[42], "cm  maxDistance:     ");
      sprintf(theNumber, "%4d", maxDistance);
      copyToBuffer(&theBuffer[58], theNumber);
      copyToBuffer(&theBuffer[62], "cm");
      Serial.println(theBuffer);
      sendMessageToSignalbox(theBuffer);
    }
  }
}

void loop()
{
  sensorID = getSensorID();

  if (haveSignalboxAddress == true)
  {
    // we have an address to send data to
    checkSignalboxMessage(); // check for messages from the signalbox
    checkManualStop();
    checkSensor();
    setDistance();
  }
  else
  {
    // waiting for an address message from the signalbox
    Serial.print(".");
    checkSignalboxMessage();
    delay(500);
  }
  if (sendAlive == true)
  {
    sendAliveMessage();
  }
}
