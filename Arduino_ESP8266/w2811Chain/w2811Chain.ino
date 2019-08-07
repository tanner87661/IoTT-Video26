
 /**lastMQTTUpload + mqttIntervalinterval
  * 
 *  ESP8266-12E / NodeMCU / WeMos D1 Mini WiFi & ENC28J60 Sample v1.0.20170110
 *  Source code can be found here: https://github.com/JZ-SmartThings/SmartThings/blob/master/Devices/Generic%20HTTP%20Device
 *  Copyright 2017 JZ
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */


//#define HasNanoAttached //activate if command should be sent out over serial interface to Nano acting as slave
//#define HasLocalChain //activate if there is also a FastLED chain connected to the board hardware
#define hasMQTT //uses roughl 18kB of RAM if activated!!!
#define APPNAME "LEDServer"
#define VERSION "V0.9.1"
#define COMPDATE __DATE__ __TIME__
#define MODEBUTTON 10

//#define isSonoff

#ifdef isSonoff
  #define pinTemp  16
  #define pinRx    5
  #define pinTx    4
//  #define pinRadar 0
  #define pinLED   2
//  #define pinIRMD  15

  #define pinChain1 14
  #define pinRelay 12
  #define pinGreenLED 13
  #define pinButton 0
#else
  #define pinTemp  D0
  #define pinRx    D1
  #define pinTx    D2
  #define pinRadar D3
  #define pinLED   D4
  #define pinIRMD  D8
  #define pinRelay 12
  #define pinGreenLED 13
  #define pinChain1 5
#endif

#include <ArduinoJson.h>

//#ifdef HasNanoAttached
  #include <SoftwareSerial.h>
  //Nano port
  SoftwareSerial nanoSerial(pinRx, pinTx, false, 1024);
  #define JSON_BUFFER_SIZE 1024
  char    jsonInput[JSON_BUFFER_SIZE];
  int     jsonInputPtr = 0;
//#endif

//#include <SD.h>
//#include "adc.h"
#include <FS.h>
#include "PayloadStruct.h"
//#ifdef HasLocalChain
  #define FASTLED_ALLOW_INTERRUPTS 0
  #include <FastLED.h>
//#endif

//#include <Time.h>
#include <TimeLib.h>
#include <Ticker.h>
#include <DHT11.h>
#include <RollAvgSmall.h>
#include <NTPtimeESP.h>
#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>

#ifdef hasMQTT
  #include <PubSubClient.h>

/*
  #include "Adafruit_MQTT.h"
  #include "Adafruit_MQTT_Client.h"

  //this is for the adafruit IoT server
  #define ARB_SERVER      "io.adafruit.com"
  #define ARB_SERVERPORT  8883                   // use 8883 for SSL
  #define ARB_USERNAME    "tanner87661"
  #define ARB_PW          "9d08e993eef84bc0b4628fa67b8427fa"
*/  
#endif

// SET YOUR NETWORK MODE TO USE WIFI
const char* ssid = "";
const char* password = "";
//const char* ssid = "TannerFamilyA";
//const char* password = "CaleraDeTanner";
String NetBIOSName = "LED_Controller";

NTPtime NTPch("us.pool.ntp.org");
const int ntpTimeout = 5000; //ms timeout for NTP request


#ifdef hasMQTT
//  WiFiClientSecure client;
WiFiClient client;
  char relayTopic[100] = "sonoff";  //ping topic, do not change. This is helpful to find Gateway IP Address if not known. 

  PubSubClient mqttClient(client);
#endif

// USE BASIC HTTP AUTH?
const bool useAuth = false;

//int DefaultWait = 380; //milliseconds
const uint32_t ntpIntervallDefault = 1800000; //half hour
const uint32_t ntpIntervallShort = 10000; //10 Seconds

#ifdef hasMQTT
  const uint32_t mqttInterval = 15000; //5 Min
#endif  

const uint32_t hubNotifyIntervall = 60000; //update hub every minute

const uint32_t sensorInterval = 250; //250ms

const uint32_t callHomeIntervall = 86400000; //change to 86400000 before deployument, check every 24 hours

volatile payload_light_controller LEDOut;

byte buttonDebounce = 0;
byte buttonStatus = 0; //released
//#ifdef HasLocalChain

typedef struct  {
  byte RGBCol[3];
  byte RepeatCycles;
} PatternEntry;

typedef struct {
  int numEntries;
  int numCycles;
  int startOfs;
  int currOfs;
  PatternEntry PatternData[4];
} Pattern;


Pattern Switzerland = {2, 10, 0, 0,  //internal loop data
                      255, 255, 255, 4, //white
                      255, 0, 0, 6}; //red

Pattern USAmerica =   {4, 15, 0, 0,
                       0, 0, 128, 6, //blue
                       255, 0, 0, 3, //red
                       255, 255, 255, 3, //white
                       255, 0, 0, 3}; //red

Pattern Christmas =   {3, 12, 0, 0, 
                       0, 200, 0, 6, //green
                       255, 0, 0, 3, //red
                       255, 255, 255, 3}; //white
                       
Pattern Colombia =    {3, 12, 0, 0,
                       255, 0, 0, 3,   //red
                       255, 165, 0, 6,  //yellow
                       0, 0, 165, 3};  //blue

Pattern Halloween =   {2, 9, 0, 0,
                       255, 128, 0, 4, //orange
                       0, 0, 0, 5}; //dark

Pattern ValentinesDay = {4, 12, 0, 0,
                       255, 255, 255, 1, //white
                       255, 0, 0, 1, //red
                       255, 0, 75, 9, //pink
                       255, 0, 0, 1}; //red

typedef void (*colProcessor)(bool initMode); 
colProcessor CurrentProcessor = NULL;
Pattern* CurrentPattern = NULL;

//#endif

//Clock g_Clock;
strDateTime dateTime;
int timeZone = -5;

char SmartHub_server[50] = "192.168.87.60";
uint16_t SmartHub_port = 39500; 

char mqtt_server[50] = "192.168.87.52";//"broker.hivemq.com"; // = Mosquitto Server IP "192.168.xx.xx" as loaded from mqtt.cfg
uint16_t mqtt_port = 1883; // = Mosquitto port number, standard is 1883, 8883 for SSL connection;
char mqtt_user[50] = "";
char mqtt_password[50] = "";

char staticGateway[50] = "192.168.87.1";
char staticNetmask[50] = "255.255.255.0";
char staticDNS[50] = "255.255.255.0";


//Ajax Command
char ajaxCmdStr[] = "/ajax_inputs";

//here comes the mesh stuff

typedef struct {
  char chainName[21]; //20 char + Terminator
  int chainType;
  int numLeds;
  int firstLed;
  int RGBCorrection;
} lightChain;

#define MAX_CHAINS 4
#define MAX_LEDS 600

//lightChain chainData[MAX_CHAINS] = {{"Chain5VP60x1", 0, 60, 0, 0xB0FFC0},{"none", -1, 0, 0, 0},{"none", -1, 0, 0, 0},{"none", -1, 0, 0, 0}};
lightChain chainData[MAX_CHAINS] = {{"none", -1, 0, 0, 0},{"none", -1, 0, 0, 0},{"none", -1, 0, 0, 0},{"none", -1, 0, 0, 0}};

DHT11 dht11(pinTemp); 

RollAvgSmall detectRadar(10);
RollAvgSmall tempDHT(20);
RollAvgSmall humiDHT(20);
RollAvgSmall ambientLight(20);
RollAvgSmall detectIRMD(10);

//constants
const uint32_t nwWaitIntervall = 1000;
const uint32_t updateEEIntervall = 60000; //wait 1 Minutes before updating EEPROM data in case more changes are coming
const uint32_t serialIntervall = 500; //transmit next data element after 500ms
#ifdef hasMQTT
  const int mqttNumVals = 5; //number of mqtt upload values
  bool useMQTTCommand = true;
#endif

// OTHER VARIALBES
uint32_t serialTimer = millis();
uint32_t hubTimer = millis();
uint32_t ntpTimer = millis();
uint32_t lastShiftTimer = millis();
uint32_t lastFrameTimer = millis();
uint32_t updateEETimer = millis();
uint32_t lastSensorUpdate = millis();

int callHomeEntry = millis();

int millisRollOver = 0;
unsigned long lastMillis = 0;
#ifdef hasMQTT
  uint32_t lastMQTTUpload = millis();
  int mqttIndexer = mqttNumVals;
#endif

bool saveEEData = false;
const int   memBase = 1024; //EEPROM location where data can be stored. Addresses below are reserved for OTA settings

Ticker myTicker;

File uploadFile;

//#ifdef HasNanoAttached
word nanoUpdateConfig = 0;
//#endif
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

//#ifdef HasLocalChain
CRGB leds[MAX_LEDS];
//#endif

String currentIP, fullrequest;

ESP8266WebServer server(80);

bool    ntpOK = false;
bool    useDHT11 = false;
bool    useRadar = false;
bool    useIRMD = false;
bool    useLight = false;
bool    useNTP = false;
bool    useUpTime = false;
bool    useMemory = false;
bool    useLocalChain = false;
bool    useNanoAttached = false;



void addLEDChain(lightChain thisChain)
{
  Serial.print("Init Chain ");
  Serial.print(thisChain.chainType);
  Serial.print(" ");
  Serial.println(thisChain.chainName);
  switch (thisChain.chainType)
  {
    case 0: FastLED.addLeds<WS2811, pinChain1, GRB>(leds, thisChain.firstLed, thisChain.numLeds).setCorrection(thisChain.RGBCorrection); break;  //used for 5V single led per node test chain
    case 1: FastLED.addLeds<WS2811, pinChain1, RGB>(leds, thisChain.firstLed, thisChain.numLeds).setCorrection(thisChain.RGBCorrection); break;  //used for 12V outdoor single led pixel chain with cables
    case 2: FastLED.addLeds<WS2811, pinChain1, BRG>(leds, thisChain.firstLed, thisChain.numLeds).setCorrection(thisChain.RGBCorrection); break;  //used for 12V outdoor triple led pixel chain
    default: break;
  }
}

void connectToWifi()
{
  WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();
    
  //set custom ip for portal
  wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  APPNAME
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(APPNAME);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Init SPIFFS");
  SPIFFS.begin();

  readNodeConfig();

  if (useNanoAttached)
    nanoSerial.begin(9600);

  Serial.println("reading EEPROM");
  readPatternConfig();
  if (!(((LEDOut.LightPattern >=0) && (LEDOut.LightPattern <= 8)) || (LEDOut.LightPattern == 99)))
  {
    LEDOut.LightPattern = 99;
    LEDOut.ShiftRate = 15;
    LEDOut.SplitChain = false; 
    LEDOut.InsideOut = true; 
    LEDOut.RandomSparks = false;
    LEDOut.RGB[0] = 255;
    LEDOut.RGB[1] = 147;
    LEDOut.RGB[2] = 41; //candle light
    LEDOut.Brightness = _max(LEDOut.RGB[0], _max(LEDOut.RGB[1], LEDOut.RGB[2]));
    writePatternConfig();
  }

  connectToWifi();

    Serial.println("");
    Serial.println("WiFi connected");
    // Start the server
    server.on("/edit", HTTP_DELETE, handleDelete);
    server.on("/edit", HTTP_POST, [](){ returnOK(); }, handleFileUpload);
    server.onNotFound(handleNotFound); //this is the default handler
    server.begin();
    Serial.println("Server started");
    // Print the IP address
    Serial.print("Use this URL to connect: ");
    Serial.print("http://"); Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.println(WiFi.macAddress());
    currentIP=WiFi.localIP().toString();

  // Start up the LED counter

  readSensorConfig();

  if (useLocalChain)
  {
    readChainConfig(); //needed for NetBiosName    
    Serial.println("Init LED Chains");
    for (int i = 0; i<MAX_CHAINS; i++)
    {
      Serial.print("Chain "); Serial.println(i);
      if (chainData[i].chainType >= 0)
        addLEDChain(chainData[i]);
    }
  
    FastLED.setBrightness(LEDOut.Brightness);

    Serial.println("Init Ticker");
    myTicker.attach_ms(33, ticker_ISR); //Frame Rate 30 Frames per Second
    Serial.println("Init Pattern");
    
    setPattern();
  }

/*
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
  interrupts();
*/  

  pinMode(pinLED, OUTPUT);
#ifndef isSonoff
  pinMode(pinRadar, INPUT);
  pinMode(pinIRMD, INPUT_PULLUP);
#endif
#ifdef isSonoff
  pinMode(pinRelay, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);
  digitalWrite(pinGreenLED, 1);
  digitalWrite(pinRelay, 0);
#endif
  Serial.println("Init WDT");
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  ESP.wdtFeed();
  Serial.println("setup complete");
}

void getInternetTime()
{
  int thisIntervall = ntpIntervallDefault;
  if (!ntpOK)
    thisIntervall = ntpIntervallShort;
  if (millis() > (ntpTimer + thisIntervall))
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("getInternetTime");
      uint32_t NTPDelay = millis();
      dateTime = NTPch.getNTPtime(timeZone, 2);
      ntpTimer = millis();
      while (!dateTime.valid)
      {
        delay(100);
        Serial.println("waiting for Internet Time");
        dateTime = NTPch.getNTPtime(timeZone, 2);
        if (millis() > NTPDelay + ntpTimeout)
        {
          ntpOK = false;
          Serial.println("Getting NTP Time failed");
          return;
        }
      }
      NTPDelay = millis() - NTPDelay;
      setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);
      ntpOK = true;
      NTPch.printDateTime(dateTime);

      String NTPResult = "NTP Response Time [ms]: ";
      NTPResult += NTPDelay;
      Serial.println(NTPResult);
    }
    else
    {
    connectToWifi();  
    }
  }
}

#ifdef hasMQTT

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  // Loop until we're reconnected --  no, not anymore, see below
  if (!mqttClient.connected()) {
    Serial.println("No MQTT connection");
    if (WiFi.status() != WL_CONNECTED) 
    {
      WiFi.disconnect();
      WiFi.reconnect();
//      wifiManager.autoConnect();
//      WiFi.begin(staName, staPassword);
    }
    Serial.print("Attempting MQTT connection..." + String(mqtt_server) + " " + String(mqtt_port) + " ");
    // Create a random client ID
    String clientId = "LEDCHAIN" + String(random(99));// + ESP_getChipId();
    Serial.println(clientId);
    // Attempt to connect
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) 
    {
      Serial.print("connected, subscribe to: ");
      Serial.println(relayTopic);
      mqttClient.subscribe(relayTopic);
    } else {
      Serial.print("failed, rc= ");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
      return; //break the loop to make sure web server can be accessed to enter a valid MQTT server address
    }
  }
//  else
//    Serial.println("MQTT ok!");
}

//called when mqtt message with subscribed topic is received
void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
//  Serial.print(topic);
//  Serial.print(" ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();

  
  char cmdBuffer[20];
  for (int i = 0; i < length; i++) 
    cmdBuffer[i] = (char)payload[i];
  cmdBuffer[length] = char(0);
  if (strcmp(cmdBuffer, "true")==0)
    digitalWrite(pinRelay, 1);
  if (strcmp(cmdBuffer, "false")==0)
    digitalWrite(pinRelay, 0);
  if (strcmp(cmdBuffer, "toggle")==0)
    digitalWrite(pinRelay, !digitalRead(pinRelay));
  digitalWrite(pinGreenLED, !digitalRead(pinRelay));
}

void uploadMQTT()
{
  int freeMem;
  mqttIndexer++;
  if (mqttIndexer > mqttNumVals)
    mqttIndexer = 0; 
/*
  switch (mqttIndexer)
  {
    if (useDHT11)
    {
      case 0: if (! temp.publish(tempDHT.average()))
                  Serial.println(F("Failed"));
              break;
      case 1: if (! humi.publish(humiDHT.average()))
                  Serial.println(F("Failed"));
              break;  
    }
    case 2: if (!brightness.publish(ambientLight.average()))
              Serial.println(F("Failed"));
            break;
    case 3: freeMem = ESP.getFreeHeap();  
            if (! memory.publish(freeMem))
              Serial.println(F("Failed"));
            break;
    case 4: if (! radarmotion.publish(detectRadar.average()))
              Serial.println(F("Failed"));
            break;
    case 5: if (! irmotion.publish(detectIRMD.average()))
              Serial.println(F("Failed"));
            break;
    default: break;  
  }
  */  
}
#endif

void loop()
{
  ESP.wdtFeed();
  if (millis() < lastMillis)
    millisRollOver++;
  else
    lastMillis = millis();  
#ifdef hasMQTT
  if (mqttClient.connected())
    mqttClient.loop();
  else
    MQTT_connect();
#endif  
  if (useNTP)
    getInternetTime();


  //-------- Your Sketch starts from here ---------------

  if (millis() > lastSensorUpdate + sensorInterval)
  {
    lastSensorUpdate = millis();
    float presVal;
#ifndef isSonoff
    if (useRadar)
    {
      presVal = digitalRead(pinRadar);
      detectRadar.update(presVal);
    }
    if (useIRMD)
    {
      presVal = digitalRead(pinIRMD);
      detectIRMD.update(presVal);
    }
    int err;
    float temp, humi;
    if (useDHT11)
    {
      if((err=dht11.read(humi, temp))==0)
      {
        tempDHT.update(temp);
        humiDHT.update(humi);
      }
    }
    if (useLight)
    {
      temp = analogRead(0);
      ambientLight.update(temp);
    }
#endif
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Try to reconnect");
    connectToWifi();
  }
  
  server.handleClient();
#ifdef isSonoff
  if (digitalRead(pinButton))
    buttonDebounce = min(buttonDebounce+1, 5);
  else
    buttonDebounce = max(buttonDebounce-1, 0);
  if ((buttonDebounce == 0) && (buttonStatus == 0)) //newly clicked
  {
      buttonStatus = 1;
      digitalWrite(pinRelay, !digitalRead(pinRelay));
      digitalWrite(pinGreenLED, !digitalRead(pinRelay));
  }
  else
    if ((buttonDebounce == 5) && (buttonStatus == 1)) //newly released
    {
      buttonStatus = 0;
    }
#endif

#ifdef hasMQTT
  if (millis() > lastMQTTUpload + mqttInterval)
  {
    uploadMQTT();
    lastMQTTUpload = millis();
  }
#endif

  if (millis() > hubNotifyIntervall + hubTimer)
  {
//    notifyHub();
    hubTimer = millis();
  }

  if (saveEEData)
    if (millis() > updateEETimer + updateEEIntervall)
    {
      int l = writePatternConfig();
      Serial.print("EE Update Len: ");
      Serial.println(l);
      saveEEData = false;
    }

//#ifdef HasNanoAttached
  if (useNanoAttached)
  {
    if (millis() > serialTimer)
    {
      handleSendData();
      serialTimer = millis() + serialIntervall;
      yield();
    }
    while (nanoSerial.available() > 0) 
    {
     jsonInput[jsonInputPtr] = nanoSerial.read();
//    Serial.print(jsonInput[jsonInputPtr]);
      if (jsonInput[jsonInputPtr] == '}')
      {
        Serial.println(jsonInput);
        handleReceiveData();
        flushJSONBuffer();
      }
      else
      {
        if ((jsonInputPtr > 0) || (jsonInput[0] == '{'))
          jsonInputPtr++;
      }
    }
  }
//#endif  

} //loop

void handleSendData()
{
  if ((nanoUpdateConfig & 0x00FF) > 0)
  {
    DynamicJsonDocument doc(200);
    if ((nanoUpdateConfig & 0x0001) > 0)
    {
      doc["LightPattern"] = LEDOut.LightPattern;
      nanoUpdateConfig &= 0xFFFE;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0002) > 0)
    {
      doc["InsideOut"] = LEDOut.InsideOut;
      nanoUpdateConfig &= 0xFFFD;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0004) > 0)
    {
      doc["SplitChain"] = LEDOut.SplitChain;
      nanoUpdateConfig &= 0xFFFB;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0008) > 0)
    {
      doc["Brightness"] = LEDOut.Brightness;
      nanoUpdateConfig &= 0xFFF7;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0010) > 0)
    {
      doc["FrameRate"] = 30; //now a constant
      nanoUpdateConfig &= 0xFFEF;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0020) > 0)
    {
      doc["ShiftRate"] = LEDOut.ShiftRate;
      nanoUpdateConfig &= 0xFFDF;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0040) > 0)
    {
      doc["RGB0"] = LEDOut.RGB[0];
      doc["RGB1"] = LEDOut.RGB[1];
      doc["RGB2"] = LEDOut.RGB[2];
      nanoUpdateConfig &= 0xFFBF;
//      return;
    }
    else if ((nanoUpdateConfig & 0x0080) > 0)
    {
      doc["RandomSparks"] = LEDOut.RandomSparks;
      nanoUpdateConfig &= 0xFF7F;
//      return;
    }
    String outMsg;
    serializeJson(doc, outMsg);
    nanoSerial.println(outMsg); 
    Serial.println(outMsg);
//    nanoUpdateConfig = 0;
  }   
}

void handleReceiveData() //receive performance and technical data from the due
{
  DynamicJsonDocument doc(200);

  DeserializationError error = deserializeJson(doc, jsonInput);
  if (error)
  {
    Serial.println("nodeMCU parseObject() failed");
    return;
  }
  if (doc.containsKey("RD"))
    if (doc["RD"] == true)
      nanoUpdateConfig = 0xFFFF;
}

void flushJSONBuffer()
{
  for (int i=0; i < JSON_BUFFER_SIZE; i++)
    jsonInput[i] = 0;
  jsonInputPtr = 0;
}

void ticker_ISR (void)
{
//  #ifdef HasLocalChain
    int timeDelta = 0;
    if (LEDOut.ShiftRate > 0)
      timeDelta = lastShiftTimer + (1000/LEDOut.ShiftRate);
    if (CurrentProcessor != NULL)
    {
      if ((millis() > timeDelta) && (timeDelta > 0))
      {
        CurrentProcessor(true);
        digitalWrite(pinLED, !digitalRead(pinLED)); 
        lastShiftTimer = millis();
      }
      else
        CurrentProcessor(false);
      FastLED.show();

    }
//  #endif
}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    EEPROM.begin(4096);
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
      EEPROM.write(ee++, *p++);
    EEPROM.commit();
//    uploadProtocolData("EEPROM updated");
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    EEPROM.begin(4096);
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    {
      *p++ = EEPROM.read(ee++);
    }
    return i;
}

int readChainConfig()
{
  DynamicJsonDocument doc(1500);
  if (SPIFFS.exists("/chain.cfg"))
  {
    File dataFile = SPIFFS.open("/chain.cfg", "r");
    if (dataFile)
    {
      Serial.print("Reading Chain Config File ");
      Serial.println(dataFile.size());
      String jsonData;
      while (dataFile.position()<dataFile.size())
      {
        jsonData = jsonData + dataFile.readStringUntil('\n');
        jsonData.trim();
      } 
      dataFile.close();
      Serial.println("Reading Chain Done");
      Serial.println(jsonData);

      DeserializationError error = deserializeJson(doc, jsonData);
      if (!error)
      {
        for (int i = 0; i < MAX_CHAINS; i++)
        {
          int intVal;
          String hlpStr = doc["chaindata" + String(i)]["chainname"];
//          Serial.println(hlpStr);
          hlpStr.toCharArray(chainData[i].chainName, sizeof(chainData[i].chainName));
          intVal = doc["chaindata" + String(i)]["chaintype"];
          chainData[i].chainType = intVal;
          intVal = doc["chaindata" + String(i)]["numleds"];
          if (intVal > MAX_LEDS)
            intVal = MAX_LEDS; //safety measure to avoid node crashing in case of memory overrun
          chainData[i].numLeds = intVal;
          intVal = doc["chaindata" + String(i)]["firstled"];
          chainData[i].firstLed = intVal;
          intVal = doc["chaindata" + String(i)]["rgbcorrection"];
          chainData[i].RGBCorrection = intVal;
/*
          Serial.println("Config read:");
          Serial.println(chainData[i].chainName);
          Serial.println(chainData[i].chainType);
          Serial.println(chainData[i].numLeds);
          Serial.println(chainData[i].firstLed);
          Serial.println(chainData[i].RGBCorrection);
*/          
        }
      }
      else
        Serial.println("Error Parsing JSON");
    }
  }
  else
    writeChainConfig();
}

int writeChainConfig()
{
  DynamicJsonDocument doc(800);
  for (int i = 0; i < MAX_CHAINS; i++)
  {
    JsonObject data = doc.createNestedObject("chaindata" + String(i));
    data["chainname"] = chainData[i].chainName;
    data["chaintype"] = chainData[i].chainType;
    data["numleds"] = chainData[i].numLeds;
    data["firstled"] = chainData[i].firstLed;
    data["rgbcorrection"] = chainData[i].RGBCorrection;
  }
  String newMsg = "";
  serializeJson(doc, newMsg);
  Serial.println(newMsg);
  Serial.println("Writing Config File");
  
  File dataFile = SPIFFS.open("/chain.cfg", "w");
  if (dataFile)
  {
    dataFile.println(newMsg);
    dataFile.close();
    Serial.println("Writing Config File complete");
  }
}

void readNodeConfig()
{
  DynamicJsonDocument doc(1500);
  if (SPIFFS.exists("/node.cfg"))
  {
    File dataFile = SPIFFS.open("/node.cfg", "r");
    if (dataFile)
    {
      Serial.print("Reading Node Config File ");
      Serial.println(dataFile.size());
      String jsonData;
      while (dataFile.position()<dataFile.size())
      {
        jsonData = jsonData + dataFile.readStringUntil('\n');
        jsonData.trim();
      } 
      dataFile.close();
      Serial.println("Reading Node Done");
      Serial.println(jsonData);
    
  // Deserialize the JSON document
      DeserializationError error = deserializeJson(doc, jsonData);
      if (!error)
      {
        if (doc.containsKey("NetBIOSName"))
        {
          String hlpStr = doc["NetBIOSName"];
          NetBIOSName = hlpStr;
        }
        if (doc.containsKey("hasLocalChain"))
          useLocalChain = bool(doc["hasLocalChain"]);
        if (doc.containsKey("hasNanoAttached"))
          useNanoAttached = bool(doc["hasNanoAttached"]);

        const char* keyData;
        
        if (doc.containsKey("SmartHub"))
        {
          keyData = doc["SmartHub"]["ip"];
          if (keyData != NULL)
            strcpy(SmartHub_server, doc["SmartHub"]["ip"]);
          keyData = doc["SmartHub"]["port"];
          if (keyData != NULL)
            SmartHub_port = uint16_t(doc["SmartHub"]["port"]);
        }
        if (doc.containsKey("useMQTTCmd"))
          useMQTTCommand = int(doc["useMQTTCmd"]);
        if (doc.containsKey("mqttServer"))
        {
          keyData = doc["mqttServer"]["ip"];
          if (keyData != NULL)
          {
            strcpy(mqtt_server, doc["mqttServer"]["ip"]);
          }
          keyData = doc["mqttServer"]["port"];
          if (keyData != NULL)
            mqtt_port = uint16_t(doc["mqttServer"]["port"]);
          keyData = doc["mqttServer"]["user"];
          if (keyData != NULL)
            strcpy(mqtt_user, doc["mqttServer"]["user"]);
          keyData = doc["mqttServer"]["password"];
          if (keyData != NULL)
            strcpy(mqtt_password, doc["mqttServer"]["password"]);
          Serial.print("MQTT Server ");
          Serial.print(mqtt_server);
          Serial.print(" Port ");
          Serial.println(mqtt_port);
        }
        if (doc.containsKey("RelayTopic"))
        {
          strcpy(relayTopic, doc["RelayTopic"]);
          Serial.println(relayTopic);
        }
      }
    }
    else
      Serial.println("File node.cfg not found");
  } 
}

void readSensorConfig()
{
  DynamicJsonDocument doc(1500);
  if (SPIFFS.exists("/sensor.cfg"))
  {
    File dataFile = SPIFFS.open("/sensor.cfg", "r");
    if (dataFile)
    {
      Serial.print("Reading Sensor Config File ");
      Serial.println(dataFile.size());
      String jsonData;
      while (dataFile.position()<dataFile.size())
      {
        jsonData = jsonData + dataFile.readStringUntil('\n');
        jsonData.trim();
      } 
      dataFile.close();
      Serial.println("Reading Sensor Done");
      Serial.println(jsonData);

      DeserializationError error = deserializeJson(doc, jsonData);

  // Test if parsing succeeds.
      if (error) 
      {
         Serial.print(F("deserializeJson() failed: "));
         Serial.println(error.c_str());
         return;
      }

      if (!error) 
      {
        if (doc.containsKey("useDHT11"))
          useDHT11 = bool(doc["useDHT11"]);
        if (doc.containsKey("useRadar"))
          useRadar = bool(doc["useRadar"]);
        if (doc.containsKey("useIRMD"))
          useIRMD = bool(doc["useIRMD"]);
        if (doc.containsKey("useLight"))
          useLight = bool(doc["useLight"]);
        if (doc.containsKey("useNTP"))
          useNTP = bool(doc["useNTP"]);
        if (doc.containsKey("ntpTimeZone"))
          timeZone = int(doc["ntpTimeZone"]);
        if (doc.containsKey("useUpTime"))
          useUpTime = bool(doc["useUpTime"]);
        if (doc.containsKey("useMemory"))
          useMemory = bool(doc["useMemory"]);
      }
    }
  } 
}

int readPatternConfig()
{
  return EEPROM_readAnything(memBase, LEDOut);
}

int writePatternConfig()
{
  return EEPROM_writeAnything(memBase, LEDOut);
}

void ProcessPattern(bool initMode)
{
    if (CurrentPattern != NULL)
      ProcessShift(CurrentPattern, initMode);
    else
      colorChange(CRGB(LEDOut.RGB[0],LEDOut.RGB[1],LEDOut.RGB[2]));
    addGlitter(80);
}

void AllDark(bool initMode)
{
  if (initMode)
    colorChange(CRGB(0,0,0));
}

//void ProcessShift(uint8_t shiftby, CRGB fillcolor, uint8_t repeat)
void ProcessShift(Pattern* ProcessThis, bool initMode)
{
  if (LEDOut.SplitChain == 1)
  {
    if (LEDOut.InsideOut == 1)
      ShiftOut(ProcessThis, initMode);
    else  
      ShiftIn(ProcessThis, initMode);
  } else
  {
    if (LEDOut.InsideOut == 1)
      ShiftUp(ProcessThis, initMode);
    else  
      ShiftDown(ProcessThis, initMode); //no split, no InsideOut
  }
}

void setFirstPatternEntry(Pattern* ofPattern, bool initMode)
{
  ofPattern->currOfs = ofPattern->startOfs;
  if (initMode)
    {
      ofPattern->startOfs++;
      if (ofPattern->startOfs >= ofPattern->numCycles)
        ofPattern->startOfs = 0;
//      lastShiftTimer = millis();
    }
}

PatternEntry* getNextPatternEntry(Pattern* ofPattern)
{
  ofPattern->currOfs++;
  if (ofPattern->currOfs >= ofPattern->numCycles)
    ofPattern->currOfs = 0;
  int PtrCtr = 0;
  int RepCtr = 0;
  while (RepCtr + ofPattern->PatternData[PtrCtr].RepeatCycles <= ofPattern->currOfs)
  {
    RepCtr += ofPattern->PatternData[PtrCtr].RepeatCycles;
    PtrCtr++;
  }
  return &ofPattern->PatternData[PtrCtr];
}

void ShiftUp(Pattern* ProcessThis, bool initMode) 
{
  int i;
  setFirstPatternEntry(ProcessThis, initMode);
  for (i = FastLED.size()-1; i >= 0; i--) 
  {
    PatternEntry* ThisEntry = getNextPatternEntry(ProcessThis);
    leds[i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
  }
}


void ShiftDown(Pattern* ProcessThis, bool initMode) {
  int i;
  setFirstPatternEntry(ProcessThis, initMode);
  for (i=0; i < FastLED.size(); i++) 
  {
    PatternEntry* ThisEntry = getNextPatternEntry(ProcessThis);
    leds[i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
  }
}

void ShiftIn(Pattern* ProcessThis, bool initMode) {
  int i, j, startup, startdown;
  bool evenval = ((FastLED.size() % 2) == 0);
  if (evenval)
  { //even # of LED's
    startup = trunc(FastLED.size() / 2);
    startdown = startup - 1;
  }
  else
  { //odd # of LED's
    startup = trunc(FastLED.size() / 2);
    startdown = startup;
  }
  setFirstPatternEntry(ProcessThis, initMode);
  for (i=0; i <= startdown; i++) 
  {  //shift up the lower half
    PatternEntry* ThisEntry = getNextPatternEntry(ProcessThis);
    leds[startdown - i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
    leds[startup + i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
  }
}

void ShiftOut(Pattern* ProcessThis, bool initMode) {
  int i, j, startup, startdown;
  bool evenval = ((FastLED.size() % 2) == 0);
  if (evenval)
  { //even # of LED's
    startup = trunc(FastLED.size() / 2);
    startdown = startup - 1;
  }
  else
  { //odd # of LED's
    startup = trunc(FastLED.size() / 2);
    startdown = startup;
  }
  setFirstPatternEntry(ProcessThis, initMode);
  int LastLED = FastLED.size() - 1; 
  for (i=0; i <= startdown; i++) 
  {  //shift up the lower half
    PatternEntry* ThisEntry = getNextPatternEntry(ProcessThis);
    leds[i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
    leds[LastLED - i] = CRGB(ThisEntry->RGBCol[0],ThisEntry->RGBCol[1],ThisEntry->RGBCol[2]);
  }
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if (LEDOut.RandomSparks)
  {
    if( random8() < chanceOfGlitter) 
      leds[ random16(MAX_LEDS) ] += CRGB::White;
  }
}

void rainbowCycle(bool initMode) 
{
  bool needUpdate = initMode;
  int i, j, startup, startdown;

  if (initMode)
  {
  gHue++;
  CHSV hsv;
  hsv.hue = gHue;
  hsv.val = 255;
  hsv.sat = 240;
  if (LEDOut.SplitChain > 0)
  {
    bool evenval = ((FastLED.size() % 2) == 0);
    if (evenval)
    { //even # of LED's
      startup = trunc(FastLED.size() / 2);
      startdown = startup - 1;
    }
    else
    { //odd # of LED's
      startup = trunc(FastLED.size() / 2);
      startdown = startup;
    }
    for (i=0; i <= startdown; i++)
    {
      if (LEDOut.InsideOut == 0)
      {
        leds[startdown - i] = hsv;
        leds[startup + i] = hsv;
      }
      else  
      {
        leds[FastLED.size()- 1 -i] = hsv;
        leds[i] = hsv;
      }
      hsv.hue += 7;
    }
  }
  else
  {
    startdown = FastLED.size() - 1;
    for (i=0; i < startdown+1; i++) 
    {
      if (LEDOut.InsideOut > 0)
        leds[i] = hsv;
      else  
        leds[startdown - i] = hsv;
      hsv.hue += 7;
    }
  }
  }
    addGlitter(80);
}

// fill the dots one after the other with said color
// good for testing purposes
void colorChange(CRGB c) {
  int i;
  for (i=0; i < FastLED.size(); i++) 
    leds[i] = c;
}

// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY
//// 
// This basic one-dimensional 'fire' simulation works roughly as follows:
// There's a underlying array of 'heat' cells, that model the temperature
// at each point along the line.  Every cycle through the simulation, 
// four steps are performed:
//  1) All cells cool down a little bit, losing heat to the air
//  2) The heat from each cell drifts 'up' and diffuses a little
//  3) Sometimes randomly new 'sparks' of heat are added at the bottom
//  4) The heat from each cell is rendered as a color into the leds array
//     The heat-to-color mapping uses a black-body radiation approximation.
//
// Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
//
// This simulation scales it self a bit depending on NUM_LEDS; it should look
// "OK" on anywhere from 20 to 100 LEDs without too much tweaking. 
//
// I recommend running this simulation at anywhere from 30-100 frames per second,
// meaning an interframe delay of about 10-35 milliseconds.
//
// Looks best on a high-density LED setup (60+ pixels/meter).
//
//
// There are two main parameters you can play with to control the look and
// feel of your fire: COOLING (used in step 1 above), and SPARKING (used
// in step 3 above).
//
// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100 
#define COOLING  55

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 120

void Fire2012(bool initMode)
{
  if (initMode)
  {

// Array of temperature readings at each simulation cell
    static byte heat[MAX_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < MAX_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / MAX_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= MAX_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < MAX_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( LEDOut.InsideOut ) {
        pixelnumber = (MAX_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }
  }
}

void confetti(bool initMode) 
{
  if (initMode)
  {
    // random colored speckles that blink in and fade smoothly
    fadeToBlackBy( leds, MAX_LEDS, 10);
    int pos = random16(MAX_LEDS);
    leds[pos] += CHSV( gHue + random8(64), 200, 255);
  }
}

void sinelon(bool initMode)
{
  if (initMode)
  {
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy( leds, MAX_LEDS, 20);
    int pos = beatsin16( 13, 0, MAX_LEDS-1 );
    leds[pos] += CHSV( gHue, 255, 192);
  }
}

void bpm(bool initMode)
{
  if (initMode)
  {
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    uint8_t BeatsPerMinute = 62;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
    for( int i = 0; i < MAX_LEDS; i++) { //9948
      leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
    }
  }
}

void juggle(bool initMode) 
{
  if (initMode)
  {
    // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy( leds, MAX_LEDS, 20);
    byte dothue = 0;
    for( int i = 0; i < 8; i++) {
      leds[beatsin16( i+7, 0, MAX_LEDS-1 )] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }
  }
}

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

void setPattern()
{ //interrupts should be disabled when calling this function
  Serial.println("Set Pattern");
  Serial.println(LEDOut.LightPattern);
  switch (LEDOut.LightPattern)
  {
    case 0: 
    {
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &USAmerica;
        break;
    }
    case 1:
    {
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &Switzerland;
        break;
    }
    case 2: 
    {
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &Colombia;
        break;
    }
    case 3: 
    {
        CurrentProcessor = &rainbowCycle;
        CurrentPattern = NULL;
        break;
    }
    case 4:
    {
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &Halloween;
        break;
    }
    case 5: 
    {
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &Christmas;
        break;
    }
    case 6: 
    { 
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = NULL;
        break;
    }
    case 7: 
    { 
        CurrentProcessor = &ProcessPattern;
        CurrentPattern = &ValentinesDay;
        break;
    }
    case 8: 
    { 
        CurrentProcessor = &Fire2012;
        CurrentPattern = NULL;
        break;
    }
    case 9: 
    { 
        CurrentProcessor = &confetti;
        CurrentPattern = NULL;
        break;
    }
    case 10: 
    { 
        CurrentProcessor = &sinelon;
        CurrentPattern = NULL;
        break;
    }
    case 11: 
    { 
        CurrentProcessor = &bpm;
        CurrentPattern = NULL;
        break;
    }
    case 12: 
    { 
        CurrentProcessor = &juggle;
        CurrentPattern = NULL;
        break;
    }
    default: 
    { 
        CurrentProcessor = &AllDark;
        CurrentPattern = NULL;
        break;
    }
  }
  if (CurrentProcessor != NULL)
    CurrentProcessor(true);
}

//==============================================================Web Server=================================================
void notifyHub()
{
  //smartHub, smartPort convert to string
  if (SmartHub_port < 0)
    return;
  Serial.println("Hub Notification: ");

  if(WiFi.status()== WL_CONNECTED)
  {   //Check WiFi connection status
    HTTPClient http;   
//    String targetIP = "http://" + SmartHub_server + ":" + SmartHub_port.toString(); //192.168.87.60:39500";
    String targetIP = "http://192.168.87.60:39500";
    Serial.println(targetIP);
    http.begin(targetIP);  //Specify destination for HTTP request
    http.addHeader("Content-Type", "application/json");             //Specify content-type header
    String reqText = handleJSON();
    http.addHeader("Content-Length", String(reqText.length()));             //Specify content-type header
    int httpResponseCode = http.POST(reqText);   //Send the actual POST request
    if(httpResponseCode > 0)
    {
      String response = http.getString();                       //Get the response to the request
      Serial.println(httpResponseCode);   //Print return code
      Serial.println(response);           //Print request answer
    }
    else
    {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end();  //Free resources
  }
  else
  {
    Serial.println("Error in WiFi connection"); 
    connectToWifi();  
  }
}

bool handleAjaxCommand(String path)
{
  Serial.print("Handle XML Command ");
  Serial.println(path);

  if (server.args() > 0)
  {
    for (int i = 0; i < server.args(); i++)
    {
//      Serial.print("POST arguments: "); Serial.println(server.args(i));
      Serial.print("Name: "); Serial.println(server.argName(i));
      Serial.print("Value: "); Serial.println(server.arg(i));
    }
  }

//step 1: analyze and handle requests from webpage
  handleAjaxRequests(path);
  
//step 2: prepare and send data update
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET");
  server.sendHeader("Access-Control-Max-Age", "3600");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Access-Control-Allow-Headers, Authorization, X-Requested-With");
  server.send(200, "application/json", handleJSON());
  return true;
}

String extractValue(String keyWord, String request)
{
  int startPos = request.indexOf(keyWord);
  int endPos = -1;
  if (startPos >= 0)
  {
    startPos = request.indexOf("=", startPos) + 1;
//    Serial.println(startPos);
    endPos = request.indexOf("&", startPos);
//    Serial.println(endPos);
    if (endPos < 0)
      endPos = request.length();
//    Serial.println(request.substring(startPos, endPos));   
    return request.substring(startPos, endPos);  
  }
  else
    return("");
}

void handleAjaxRequests(String request) 
{
  Serial.println(request);
  bool changedData = false;
  String hlpStr;
  if (request.indexOf("RebootNow") != -1)  
  {
    #ifdef hasMQTT
//      if (! protocol.publish("Rebooting Light Server"))
//        Serial.println(F("Failed"));
    #endif
    Serial.println("Calling Reboot");
    while(true);
  }

  int OldValue = LEDOut.LightPattern;
  if (request.indexOf("Pattern=") != -1) 
    LEDOut.LightPattern = extractValue("Pattern=", request).toInt();
  if (LEDOut.LightPattern != OldValue) 
  {
    switch (LEDOut.LightPattern)
    {
      case 98: LEDOut.LightPattern = LEDOut.PrevPattern; break;
      case 99: break;
      default: LEDOut.PrevPattern = LEDOut.LightPattern; break;
    }
      
    changedData = true;
    #ifdef hasMQTT
//      if (! protocol.publish("New Pattern: "))
//        Serial.println(F("Failed"));
    #endif    
    if (useNanoAttached)
      nanoUpdateConfig |= 0x0001;
    if (useLocalChain)
      setPattern();
  }

  bool OldStatus = LEDOut.InsideOut;
  if (request.indexOf("InsideOut=") != -1) 
  {
    hlpStr = extractValue("InsideOut=", request);
    hlpStr.toLowerCase();
    LEDOut.InsideOut = hlpStr.equals("true");
  }
  if (LEDOut.InsideOut != OldStatus) 
  {
    changedData = true;
    #ifdef hasMQTT
//      if (! protocol.publish("New InsideOut Status"))
//        Serial.println(F("Failed"));
    #endif
    if (useNanoAttached)
//   #ifdef HasNanoAttached
      nanoUpdateConfig |= 0x0002;
//    #endif
  }

  OldStatus = LEDOut.SplitChain;
  if (request.indexOf("SplitChain=") != -1) 
  {
    hlpStr = extractValue("SplitChain=", request);
    Serial.println(hlpStr);
    hlpStr.toLowerCase();
    LEDOut.SplitChain = hlpStr.equals("true");
  }
  if (LEDOut.SplitChain != OldStatus) 
  {
    changedData = true;
//    #ifdef HasNanoAttached
    if (useNanoAttached)
      nanoUpdateConfig |= 0x0004;
//    #endif
  }

  OldStatus = LEDOut.RandomSparks;
  if (request.indexOf("SelectSparks=") != -1) 
  {
    hlpStr = extractValue("SelectSparks=", request);
    hlpStr.toLowerCase();
    LEDOut.RandomSparks = hlpStr.equals("true");
  }
  if (LEDOut.RandomSparks != OldStatus) 
  {
    changedData = true;
//    #ifdef HasNanoAttached
    if (useNanoAttached)
      nanoUpdateConfig |= 0x0080;
//    #endif
  }

  OldValue = LEDOut.ShiftRate;
  if (request.indexOf("ShiftRate=") != -1) 
    LEDOut.ShiftRate = extractValue("ShiftRate=", request).toInt();
  if (LEDOut.ShiftRate != OldValue) 
  {
    changedData = true;
//    #ifdef HasNanoAttached
    if (useNanoAttached)
      nanoUpdateConfig |= 0x0020;
//    #endif
  }

  if (request.indexOf("RGB=") != -1) 
  {
    String value = extractValue("RGB=", request);
    OldValue = LEDOut.RGB[0];
    LEDOut.RGB[0] = hexToDec(value.substring(0,2));
    if (LEDOut.RGB[0] != OldValue) 
    {
      changedData = true;
//      #ifdef HasNanoAttached
      if (useNanoAttached)
        nanoUpdateConfig |= 0x0040;
//      #endif
    }
    OldValue = LEDOut.RGB[1];
    LEDOut.RGB[1] = hexToDec(value.substring(2,4));
    if (LEDOut.RGB[1] != OldValue) 
    {
      changedData = true;
//      #ifdef HasNanoAttached
      if (useNanoAttached)
        nanoUpdateConfig |= 0x0040;
//      #endif
    }
    OldValue = LEDOut.RGB[2];
    LEDOut.RGB[2] = hexToDec(value.substring(4)); //until the end
    if (LEDOut.RGB[2] != OldValue) 
    {
      changedData = true;
//      #ifdef HasNanoAttached
      if (useNanoAttached)
        nanoUpdateConfig |= 0x0040;
//      #endif
    }
  }

  OldValue = LEDOut.Brightness;
//  LEDOut.Brightness = _max(LEDOut.RGB[0], _max(LEDOut.RGB[1], LEDOut.RGB[2]));
  if (request.indexOf("Brightness=") != -1) //0..255
    LEDOut.Brightness = extractValue("Brightness=", request).toInt();
  if (LEDOut.Brightness != OldValue) 
  {
    changedData = true;
//    #ifdef HasNanoAttached
    if (useNanoAttached)
      nanoUpdateConfig |= 0x0008;
//    #endif  
    if (useLocalChain)
      FastLED.setBrightness(LEDOut.Brightness);
  }
  if (changedData)
  {
    saveEEData = true;
    updateEETimer = millis();
    Serial.println("Ajax Request -> Update EEPROM");
  }
}

String handleJSON()
{
  String response;
  float float1;
  long curTime = now();
  DynamicJsonDocument doc(500);
  doc["IP"] = currentIP;
  if (WiFi.status() == WL_CONNECTED)
  {
    long rssi = WiFi.RSSI();
    doc["SigStrength"] = rssi;
  }
  if (useMemory)
    doc["mem"] = ESP.getFreeHeap();
  if (useUpTime)
  {  
    float1 = (millisRollOver * 4294967.296) + millis()/1000;
    doc["uptime"] = round(float1);
  }
  if (ntpOK)
  {
    if (NTPch.daylightSavingTime(curTime))
      curTime -= (3600 * (timeZone+1));
    else
      curTime -= (3600 * timeZone);
    doc["currenttime"] = curTime;  //seconds since 1/1/1970
  }
  if (useLight)
    doc["daylight"] =  ambientLight.average();
  if (useDHT11)
  {
    doc["temp"] = tempDHT.average();
    doc["hum"] = humiDHT.average();
  }
  if (useRadar)
    doc["radar"] = detectRadar.average();
  if (useIRMD)
    doc["irmd"] = detectIRMD.average();
  doc["pattern"] = LEDOut.LightPattern;
  doc["shiftrate"] = LEDOut.ShiftRate;
  doc["split"] = LEDOut.SplitChain;
  doc["insideout"] = LEDOut.InsideOut;
  doc["sparks"] = LEDOut.RandomSparks;
  for (int i=0; i<3; i++)
  {
//    String hlpStr = LEDOut.RGB[i];
//    while (hlpStr.length() < 2)
//      hlpStr = '0' + hlpStr;
    doc["RGB"+String(i)] = LEDOut.RGB[i];  
  }  
  doc["Brightness"] = LEDOut.Brightness;

  serializeJson(doc, response);
  
//  doc.printTo(response);
  Serial.println(response);
  return response;
}

void returnOK() {
  server.send(200, "text/plain", "");
}

void returnFail(String msg) {
  server.send(500, "text/plain", msg + "\r\n");
}


bool loadFromSdCard(String path){
  String dataType = "text/plain";
  if(path.endsWith("/")) path += "index.htm";

  Serial.print("Load from SPIFFS - Path: ");
  Serial.println(path);

  
  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";

//  Serial.print("opening: ");
//  Serial.println(path.c_str());
  File dataFile = SPIFFS.open(path.c_str(), "r");
//  if(dataFile.isDirectory()){
//    path += "/index.htm";
//    dataType = "text/html";
//    dataFile = SPIFFS.open(path.c_str());
//  }

  if (!dataFile)
  {
    Serial.println("File not found");
    return false;
  }

  if (server.hasArg("download")) dataType = "application/octet-stream";
  Serial.println(dataFile.size());
//  Serial.println(dataType);

//  char buf[1024];
  int siz = dataFile.size();

  int i = server.streamFile(dataFile, dataType);
  if (i != dataFile.size()) 
  {
    Serial.println(i);
    Serial.println("Sent less data than expected!");
  }
  
/*  while(siz > 0) 
  {
    size_t len = std::min((int)(sizeof(buf) - 1), siz);
    dataFile.read((uint8_t *)buf, len);
//    server.client().write((const char*)buf, len);
    server.sendContent_P((const char*)buf, len);
//    yield;
    siz -= len;
    Serial.println(siz);
  }
*/
  Serial.println("all sent");
  dataFile.close();
  return true;
}

void handleFileUpload()
{
  Serial.println("Handle Upload");
  Serial.println(server.uri());
  if(server.uri() != "/edit") 
    return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START)
  {
    if(SPIFFS.exists((char *)upload.filename.c_str())) 
    {
      Serial.print("Delete existing file "); Serial.println(upload.filename);
    }
    String hlpStr = "/" + upload.filename;
    uploadFile = SPIFFS.open(hlpStr, "w");
    if (!uploadFile)
      Serial.print("Creation of file failed");
    else
      Serial.print("Upload: START, filename: "); Serial.println(upload.filename);
  } 
  else 
    if(upload.status == UPLOAD_FILE_WRITE)
    {
      if (uploadFile) 
      {
        uploadFile.write(upload.buf, upload.currentSize);
        Serial.print("Upload: WRITE, Bytes: "); Serial.println(upload.currentSize);
      }
      else
        Serial.println("Write operation failed");
    } 
    else 
      if(upload.status == UPLOAD_FILE_END)
      {
        if (uploadFile)
        { 
          uploadFile.close();
          Serial.print("Upload: END, Size: "); Serial.println(upload.totalSize);
        }
        else
          Serial.println("Closing failed");
      }
      else
      {
        Serial.print("Unknown File Status "); Serial.println(upload.status);
      }
}

void deleteRecursive(String path){
/*  File file = SD.open((char *)path.c_str());
  if(!file.isDirectory()){
    file.close();
    SD.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while(true) {
    File entry = file.openNextFile();
    if (!entry) break;
    String entryPath = path + "/" +entry.name();
    if(entry.isDirectory()){
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SD.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD.rmdir((char *)path.c_str());
  file.close();
*/  
}

void handleDelete()
{
  String path = server.arg(0);
  Serial.print("Trying to delete ");
  Serial.println((char *)path.c_str());
  if(server.args() == 0) return returnFail("BAD ARGS");
  if(path == "/" || !SPIFFS.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }
//  deleteRecursive(path);
  returnOK();
}

void handleCreate(){
/*
  if(server.args() == 0) return returnFail("BAD ARGS");
  String path = server.arg(0);
  if(path == "/" || SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }

  if(path.indexOf('.') > 0){
    File file = SD.open((char *)path.c_str(), FILE_WRITE);
    if(file){
      file.write((const char *)0);
      file.close();
    }
  } else {
    SD.mkdir((char *)path.c_str());
  }
  */
  returnOK();
}

void printDirectory() {
/*
  if(!server.hasArg("dir")) return returnFail("BAD ARGS /list");
  String path = server.arg("dir");
  if(path != "/" && !SD.exists((char *)path.c_str())) return returnFail("BAD PATH");
  File dir = SD.open((char *)path.c_str());
  path = String();
  if(!dir.isDirectory()){
    dir.close();
    return returnFail("NOT DIR");
  }
  dir.rewindDirectory();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/json", "");
  WiFiClient client = server.client();

  server.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry)
    break;

    String output;
    if (cnt > 0)
      output = ',';

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.name();
    output += "\"";
    output += "}";
    server.sendContent(output);
    entry.close();
 }
 server.sendContent("]");
 dir.close();
 */
}

void handleNotFound(){
//this is the hook to handle async requests
  Serial.println(server.uri());
  if ((server.uri().indexOf(ajaxCmdStr) != -1) && handleAjaxCommand(server.uri())) {return; }
//this is the default file handler
  if(loadFromSdCard(server.uri())) {return;}
  String message = "SDCARD Not Detected or File not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}
