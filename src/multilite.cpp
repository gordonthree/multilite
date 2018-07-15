#include <Arduino.h>
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include <EasyNTPClient.h>
#include <FS.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "DallasTemperature.h"
#include "pca9633.h"
#include "Adafruit_ADS1015.h"
#include <Wire.h>
#include <DHTesp.h>

// uncomment for ac switch module, leave comment for dc switch module
// #define _ACMULTI true
// #define _TRAILER true
// owdat is set by json config now!

#ifdef _ACMULTI // driving relay modules, 0 is on, 1 is off
  #define _ON 0
  #define _OFF 1
  //#define OWDAT 4 // owdat usually on 4 for ac nodes, 13 for dc nodes
  ADC_MODE(ADC_VCC);
#else //driving mosfets, 1 is on, 0 is off
  #define _ON 1
  #define _OFF 0
// ADC_MODE(ADC_VCC); // added for outdoor probe
// #define OWDAT 13 // dc nodes are usualy using 13 for owdat, outdoor probes use 4
// ADC_MODE(ADC_VCC); // add for outdoor probe, rgbled module
#endif

#ifdef _TRAILER // iot node for rv application
char ntpServerName[32] = "192.168.10.30";
char* iotSrv = "192.168.10.30"; // automation api server name
#else // iot node for home application
char ntpServerName[32] = "us.pool.ntp.org";
char* iotSrv = "192.168.2.30"; // automation api server name
#endif

#ifdef _PWMC // SDK based PWM code
extern "C"{
	#include "pwm.h"
	#include "user_interface.h"
}

////// PWM

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz

#define PWM_PERIOD 5000 // this is how many steps from full off to full on. less steps means higher freq

// PWM channels

#define PWM_CHANNELS 3

// PWM setup (choice all pins that you use PWM)

uint32 io_info[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
	{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
	{PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
//	{PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
	{PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
//	{PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D5
//	{PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D6
//	{PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D7
//	{PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 ,15}, // D8
											   // D0 - not have PWM :-(
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];
#endif

const char* logFileName = "/log.csv"; // spiffs system log file name
const char* cfgFileName = "/iot.json"; // config filename (json config)
const int jsonSize = 1024; // memory to set aside to store json data

#define ADC 0x49
#define ADCPWR 0x20
#define OFF 0x0
#define ON 0x1
#define IODIR 0x00
#define GPIO 0x09
#define DHTPIN 14 // DHT11 always on GPIO 14 for now?

char foo[8];
//char rssiChr[10];
char myChr[32];
//char voltsChr[10];
//char amps0Chr[10];
//char amps1Chr[10];
//char adcChr[10];
char rebootChar[100];

float amps0 = 0.0, amps1 = 0.0, volts0 = 0.0;
int16_t adcVal[4];
uint8_t adcEnable = 0;
uint8_t wifiDown = 0;
unsigned char wsConcount=0;
int raw0=0, raw1=0, raw2=0;
char tmpChr[10];
unsigned char mac[6];
char macStr[12];
char url[100];
char str[64];
char sw1label[32], sw2label[32], sw3label[32], sw4label[32];
char nodename[32];
char vdivsor[8];
int OWDAT=-1; //
int  mqttport=0;
char mqttServer[32], mqttpub[64],mqttsub[64], mqttbase[64];
char fwversion[6]; // storage for sketch image version
char fsversion[6]; // storage for spiffs image version
char theURL[128];
char i2cbuff[30];
unsigned char sw1type=0, sw2type=0, sw3type=0, sw4type=0; // 0=swtich, 1=amps only, 2=rgbw
char fileName[32] = { 0 };
char fileURL[100] = { 0 };
float vccDivisor = 16.306;
float mvPerA = 44.0;
int iotPort = 3000; // automation api tcp port
int httpVer = 0; // storage for online config version
int fsVer = 0; // storage for fs config version
int sleepPeriod = 900;
int vccOffset = 0;
int ACSoffset = 1641;
int adc0 = 0;
uint16_t updateRate = 30;
int tstatSet = 21; // default to 21c roughly 70f
uint8_t tstatMode=0, tstatOper=0; // default thermostat off
int tstatAmb = 0; // ambient temperature
uint8_t tstatRh = 0; // rel humidity
time_t oldEpoch = 0;
uint8_t red=0,green=0,blue=0,white=0;
uint8_t oldred=0,oldgreen=0,oldblue=0,oldwhite=0;
uint8_t rgbwChan=57; // b00111001 four nibbles to map RGBW to pwm channels
uint8_t updateCnt = 0;
unsigned char newWScon = 0;
unsigned char mqttFail = 0;
unsigned char speedAddr = 0; // i2c address for speed control chip
unsigned char fanSpeed=0, fanDirection=0;
int sw1 = -1, sw2 = -1, sw3 = -1, sw4 = -1;
bool rebootMQTT = true; // flag to signal reboot on mqtt broker unavailable
bool altAdcvbat = false;
bool safeMode = false;
bool getTime = false;
bool hasRGB = false;
bool hasSerial = false;
bool fileSet = false;
bool firstBoot = true;
bool clientCon = false; // flag for websock connection
bool useMQTT = false; // flag for mqtt available
bool setPolo = false;
bool doUpdate = false;
bool getRGB = false;
bool prtLog = false;
bool skipSleep = false; // skip next sleep cycle
bool sleepEn = false; // disable sleep entirely
bool useGetvcc = false; // use internal divider network
bool hasTstat = false; // thermostat function disabled
bool hasTout = false; // output dallas temperature
bool hasIout = false; // output ADS current readings
bool hasVout = false; // output voltage / onboard adc
bool hasSpeed = false; // has pwm speed control chip (unimplemented)
bool hasRSSI = false; // output RSSI
bool hasFan = false; // flag for fan controller
bool hasDimmer = false; // flag for pwm dimming support
unsigned char hasTpwr = false; // has dallas power control (pin number)
bool hasI2C = false; // has i2c bus
bool hasI2Cpwr = false; // has i2c bus power control
bool rawadc = false; // output raw internal adc reading
bool setReset = false; // flag for reboot
bool hasHostname = false; // flag for hostname being set by saved config
bool scanI2C = false;
bool rgbTest = false;
bool prtTstat = false; // flag to request tstat mode print
bool prtConfig = false; // flag to request config print via mqtt
bool timeOut = false; // flag to report time via mqtt
bool hasADC = false; // flag for ads1015 support
bool coldBoot = true; // assume every boot is cold unless told otherwise
bool rmLog = false; // flag to remove spiffs log file
bool rmConfig = false; // flag to remove spiffs config file
unsigned char ntpOffset = 4; // offset from GMT
uint8_t iotSDA = 12, iotSCL = 14; // i2c bus pins
long loopTimer = 0;
long lastReconnectAttempt = 0;

int ch1en = -1, ch2en = -1, ch3en = -1, ch4en = -1;

Adafruit_ADS1115 ads;
WiFiClient espClient;
PubSubClient mqtt(espClient);
OneWire oneWire;
DallasTemperature ds18b20 = NULL;
WebSocketsServer webSocket = WebSocketsServer(81);
PCA9633 rgbw; // instance of pca9633 library
DHTesp dht; // instance of DHT library

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */

EasyNTPClient ntpClient(udp, ntpServerName);

void i2c_wordwrite(uint8_t address, uint8_t cmd, uint16_t theWord) {
  //  Send output register address
  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.write(highByte(theWord));  //  high byte
  Wire.write(lowByte(theWord));  //  send low byte of word data
  Wire.endTransmission();
}

void i2c_write(uint8_t address, uint8_t cmd, uint8_t data) {
  //  Send output register address
  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.write(data);  //  send byte data
  Wire.endTransmission();
}

uint8_t i2c_read(uint8_t address, uint8_t cmd) {
  byte result;

  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t) 1); // request two bytes
  result = Wire.read();

  return result;
}

uint8_t i2c_wordread(uint8_t address, uint8_t cmd) {
  uint16_t result;
  uint8_t xlo, xhi;

  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t) 2); // request two bytes
  xhi = Wire.read();
  xlo = Wire.read();

  result = xhi << 8; // hi byte
  result = result | xlo; // add in the low byte

  return result;
}

void i2c_readbytes(uint8_t address, uint8_t cmd, uint8_t bytecnt) {

  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.endTransmission();

  Wire.requestFrom(address, bytecnt); // request cnt bytes
  for (byte x = 0; x < bytecnt; x++) {
    i2cbuff[x] = Wire.read();
  }
}

void mqttPrintStr(char* _topic, char* myStr) {
  if (!useMQTT) return; // abort if mqtt not setup
  char myTopic[255];
  sprintf(myTopic, "%s/%s\0", mqttbase, _topic);
  mqtt.publish(myTopic, myStr);
}

void mqttPrintInt(char* myTopic, int myNum) {
  char myStr[16];
  sprintf(myStr, "%d", myNum);
  mqttPrintStr(myTopic, myStr);
}

void mqttPrintFloat(char* myTopic, float myNum) {
  char myStr[16];
  sprintf(myStr, "%f", myNum);
  mqttPrintStr(myTopic, myStr);
}

void printIOTaddr() {
  sprintf(str,"Using IOT server %s:%u", iotSrv, iotPort);
  mqttPrintStr(mqttpub, str);
}

void printIOTurl() {
  sprintf(str,"URL http://%s:%u%s", iotSrv, iotPort, theURL);
  mqttPrintStr("http_update", str);
}

void printMQTTaddr() {
  sprintf(str,"Using MQTT server %s.", mqttServer);
  mqttPrintStr(mqttpub, str);
}

void printNTPaddr() {
  sprintf(str,"Using NTP server %s.", ntpServerName);
  mqttPrintStr("time", str);
}

void printReboot() {
  mqttPrintStr("reboot/reason", rebootChar);
}

void printSwitches() {
  char switches[100];
  sprintf(switches, "pin assignments sw1=%d sw2=%d sw3=%d sw4=%d\0", sw1, sw2, sw3, sw4);
  mqttPrintStr("debug",switches);
}

void wsSend(const char* _str) {
  if (sizeof(_str)<=1) return; // don't send blank messages
  if (wsConcount>0) {
    for (int x=0; x<wsConcount; x++) {
      webSocket.sendTXT(x, _str);
    }
  }
}

void i2c_scan() {
  scanI2C = false;
  uint8_t error, address;
  uint8_t nDevices;

  mqttPrintStr(mqttpub, "Scanning I2C Bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      sprintf(str,"i2c dev %d: %x", nDevices, address);
      wsSend(str);
      if (useMQTT) {
        mqttPrintStr(mqttpub, str);
        mqtt.loop();
      }
      delay(10);
      nDevices++;
    }
  }
  mqttPrintStr(mqttpub, "I2C scan complete.");
}

void writeLog(const char* _event, const char* _message) {
  time_t ts = now();

  File logFile = SPIFFS.open(logFileName,"a");
  if (!logFile) {
    return; // oh well we tried
  }

  // record something here
  if (ts<10000000) {
    logFile.print("time not set");
  } else {
    logFile.print(ts);
  }
  logFile.print(',');
  logFile.print(_event); logFile.print(',');
  logFile.println(_message);
  logFile.close();
}

void deleteLog() {
  rmLog = false;
  if (SPIFFS.remove(logFileName)) mqttPrintStr("log", "Log file removed");
  writeLog("system","log file removed");
}

void deleteConfig() {
  rmConfig = false;
  if (SPIFFS.remove(cfgFileName)) {
    mqttPrintStr("config", "Config file removed");
    writeLog("system","config file removed");
  } 
  
}

void readLog() {
  const uint8_t bSize=240;
  const char endLine = '\n';
  char logLine[bSize];
  char tempStr[bSize+10];
  prtLog = false;

  memset(logLine,0,sizeof(logLine));
  memset(tempStr,0,sizeof(tempStr));

  mqttPrintStr("log", "Attempting to open log file");
  File logFile = SPIFFS.open(logFileName,"r");
  if (!logFile) {
    mqttPrintStr("log", "Failed to open log file");
    return; // oh well we tried
  }
  // mqttPrintStr("log", "Log file opened");
  int c = 0;
  while (logFile.available()) {
    int readBytes = logFile.readBytesUntil(endLine, logLine, bSize);
    sprintf(tempStr,"line %u: %s\0", c, logLine);
    mqttPrintStr("log", tempStr);
    c++;
  }
  mqttPrintStr("log", "End of log");
  logFile.close();
  deleteLog();
}


void httpUpdater() {
  printIOTurl();
  mqttPrintStr("http_update", "Checking...");
  //writeLog("http_update", "checking");
  char tempStr[120];
  char errStr[100];
  memset(tempStr,0,sizeof(tempStr));

  t_httpUpdate_return ret = ESPhttpUpdate.update(iotSrv, iotPort, theURL, fwversion);
 
  int errCode = ESPhttpUpdate.getLastError(); // get error code
  String errTxt = ESPhttpUpdate.getLastErrorString(); // get error message as Arduino string
  errTxt.toCharArray(errStr, errTxt.length()+1); // copy error message to char array
  sprintf(tempStr,"error %d:%s\0", errCode, errStr);

  switch(ret) {
      case HTTP_UPDATE_FAILED:
        writeLog("http_update", tempStr);
        mqttPrintStr("http_update", tempStr);
        delay(10);
        break;

      case HTTP_UPDATE_NO_UPDATES:
        //writeLog("http_update","no update available");
        mqttPrintStr("http_update", "No update available");
        delay(10);
        break;

      case HTTP_UPDATE_OK:
        writeLog("http_update","complete");
        break;
  }
}

void wsSendTime(const char* msg, time_t mytime) {
  memset(str,0,sizeof(str));
  sprintf(str, msg, mytime);
  wsSend(str);
}

void wsSendStr(const char* label, const char* msg) {
  memset(str,0,sizeof(str));
  sprintf(str, "%s=%s", label, msg);
  wsSend(str);
}

void wsSendInt(const char* label, int num) {
  memset(str,0,sizeof(str));
  sprintf(str, "%s=%d", label, num);
  wsSend(str);
}

void wsSendFloat(const char* label, float num) {
  memset(str,0,sizeof(str));
  sprintf(str, "%s=%f", label, num);
  wsSend(str);
}

void speedControl(uint8_t fanSpeed, uint8_t fanDirection) {
  if (!hasI2C) return; // bail out if i2c not setup

  //sprintf(str,"Fan %u dir %u speed %u", speedAddr, fanDirection, fanSpeed);
  //if (useMQTT) mqtt.publish(mqttpub, str);

  if (speedAddr>0) {
    i2c_write(speedAddr, 0x03, fanDirection);
    delay(10);
    i2c_write(speedAddr, 0x00, fanSpeed);
  }
}

int loadConfig(bool setFSver) {
  int ver = -1;
  File configFile = SPIFFS.open(cfgFileName, "r");
  if (!configFile) {
    return ver;
  }

  size_t size = configFile.size();
  if (size > jsonSize) {
    configFile.close();
    return ver;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);
  configFile.close();

  StaticJsonBuffer<jsonSize> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    return ver;
  }

  // uncomment to test exception handling ability
  // strcpy(foo, json["foobar"]);

  if (json.containsKey("nodename")) {
    hasHostname=true;
    strcpy(nodename, json["nodename"]);
  }

  if (json.containsKey("iotserver"))  strcpy(iotSrv, json["iotserver"]); // replace default IOT server with new one if provided
  if (json.containsKey("sw1label"))   strcpy(sw1label, json["sw1label"]);
  if (json.containsKey("sw2label"))   strcpy(sw2label, json["sw2label"]);
  if (json.containsKey("sw3label"))   strcpy(sw3label, json["sw3label"]);
  if (json.containsKey("sw4label"))   strcpy(sw4label, json["sw4label"]);
  if (json.containsKey("mqttserver")) strcpy(mqttServer, json["mqttserver"]);
  if (json.containsKey("ntpserver")) strcpy(ntpServerName, json["ntpserver"]);
  if (json.containsKey("vccdivsor"))  vccDivisor = atof((const char*)json["vccdivsor"]);
  if (json.containsKey("mvpera"))     mvPerA = atof((const char*)json["mvpera"]);

  if (json.containsKey("mqttbase")) {
    const char* mbase = json["mqttbase"];
    const char* msub = json["mqttsub"];
    const char* mpub = json["mqttpub"];
    sprintf(mqttbase, "%s/%s", mbase, nodename);
    sprintf(mqttsub, "%s/%s", mqttbase, msub);
    sprintf(mqttpub, "%s", mpub);
  }

  sleepEn = json["sleepenable"];
  sleepPeriod = json["sleepperiod"];
  mqttport = json["mqttport"];
  ver = json["cfgversion"];
  useGetvcc = json["usegetvcc"];
  hasRGB = json["hasrgb"];
  hasTout = json["hastout"];
  hasVout = json["hasvout"];
  hasIout = json["hasiout"];
  hasSpeed = json["hasspeed"];
  speedAddr = json["speedaddr"];
  hasRSSI = json["hasrssi"];
  hasTstat = json["haststat"];
  hasTpwr = json["hastpwr"]; // also serves as OWPWR
  OWDAT = json["owdat"]; // set onewire data pin
  hasI2C = json["hasi2c"];
  hasADC = json["hasadc"];
  rawadc = json["rawadc"];
  hasI2Cpwr = json["hasi2cpwr"];
  iotSDA = json["iotsda"];
  iotSCL = json["iotscl"];
  ntpOffset = json["ntpoffset"];
  ACSoffset = json["acsoffset"];
  vccOffset = json["vccoffset"];
  updateRate = json["updaterate"];
  altAdcvbat = json["altadcvbat"];
  hasFan = json["hasfan"];
  rgbwChan = json["rgbwchan"];
  timeOut = json["timeout"];
  tstatMode = json["tstatmode"];

  if (firstBoot) { // only do this at startup, resetting switches to database values
    // setup switch pins
    sw1 = json["sw1pin"];
    sw1type = json["sw1type"];
    sw2 = json["sw2pin"];
    sw2type = json["sw2type"];
    sw3 = json["sw3pin"];
    sw3type = json["sw3type"];
    sw4 = json["sw4pin"];
    sw4type = json["sw4type"];

    byte tempsw1 = json["sw1en"];
    byte tempsw2 = json["sw2en"];
    byte tempsw3 = json["sw3en"];
    byte tempsw4 = json["sw4en"];

    if (sw1>=0) {
      ch1en=tempsw1;
      pinMode(sw1, OUTPUT);
      digitalWrite(sw1, tempsw1^_OFF);
    }
    if (sw2>=0) {
      ch2en=tempsw2;
      pinMode(sw2, OUTPUT);
      digitalWrite(sw2, tempsw2^_OFF);
    }
    if (sw3>=0) {
      ch3en=tempsw3;
      pinMode(sw3, OUTPUT);
      digitalWrite(sw3, tempsw3^_OFF);
    }
    if (sw4>=0) {
      ch4en=tempsw4;
      pinMode(sw4, OUTPUT);
      digitalWrite(sw4, tempsw4^_OFF);
    }
    firstBoot = false;
  }

  if (setFSver) {
    if (json.containsKey("fwversion")) strcpy(fwversion, json["fwversion"]);
  }

  return ver;
}

void doReset() { // reboot on command
      mqttPrintStr(mqttpub, "Rebooting!");
      delay(50);
      writeLog("reboot","user request");
      ESP.reset();
      delay(5000); // allow time for reboot
}

void wsSendlabels(uint8_t _num) { // send switch labels only to newly connected websocket client
  memset(str,0,sizeof(str));
  sprintf(str,"sending labels: sw1=%d %d sw2=%d %d sw3=%d %d sw4=%d %d",sw1,sw1type,sw2,sw2type,sw3,sw3type,sw4,sw4type);
  mqttPrintStr(mqttpub, str);
  wsSend(str);
  char labelStr[8];
  if (sw1>=0) {
    if (sw1type==0) strcpy(labelStr,"switch\0"); // plain on-off switch
    else if (sw1type==1) strcpy(labelStr,"label\0"); // something other than a switch
    else if (sw1type==2) strcpy(labelStr,"rgb\0"); // rgb control
    else if (sw1type==3) strcpy(labelStr,"adc\0");
    else if (sw1type>=8) strcpy(labelStr,"fan\0"); // fan speed control
    sprintf(str,"%s=%s",labelStr, sw1label);
    wsSend(str);
  }
  if (sw2>=0) {
    if (sw2type==0) strcpy(labelStr,"switch\0");
    else if (sw2type==1) strcpy(labelStr,"label\0");
    else if (sw2type==2) strcpy(labelStr,"rgb\0");
    else if (sw2type==3) strcpy(labelStr,"adc\0");
    sprintf(str,"%s=%s",labelStr, sw2label);
    wsSend(str);
  }
  if (sw3>=0) {
    if (sw3type==0) strcpy(labelStr,"switch\0");
    else if (sw3type==1) strcpy(labelStr,"label\0");
    else if (sw3type==2) strcpy(labelStr,"rgb\0");
    else if (sw3type==3) strcpy(labelStr,"adc\0");
    sprintf(str,"%s=%s",labelStr, sw3label);
    wsSend(str);
  }
  if (sw4>=0) {
    if (sw4type==0) strcpy(labelStr,"switch\0");
    else if (sw4type==1) strcpy(labelStr,"label\0");
    else if (sw4type==2) strcpy(labelStr,"rgb\0");
    else if (sw4type==3) strcpy(labelStr,"adc\0");
    sprintf(str,"%s=%s",labelStr, sw4label);
    wsSend(str);
  }
  newWScon = 0;
}

void wsSwitchstatus() {
  char swChr[7];
  if (newWScon>0) wsSendlabels(newWScon);
  memset(swChr,0,sizeof(swChr));
  if (ch1en>=0) {
    sprintf(swChr,"sw1=%u",ch1en);
    wsSend(swChr);
  }
  if (ch2en>=0) {
    sprintf(swChr,"sw2=%u",ch2en);
    wsSend(swChr);
  }
  wsSend(swChr);
  if (ch3en>=0) {
    sprintf(swChr,"sw3=%u",ch3en);
    wsSend(swChr);
  }
  if (ch4en>=0) {
    sprintf(swChr,"sw4=%u",ch4en);
    wsSend(swChr);
  }

}

int requestConfig(bool save) {
  int ret = -1;

  HTTPClient http;

  // configure url
  sprintf(url, "http://%s:%d/iotconfig?mac=%s", iotSrv, iotPort, macStr);
  http.begin(url); //HTTP

  // start connection and send HTTP header
  int httpCode = http.GET();

  // httpCode will be negative on error
  if(httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      // file found at server
      if(httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
          int paysize = payload.length() + 1;
          char json[paysize];

          payload.toCharArray(json, paysize);

          StaticJsonBuffer<jsonSize> jsonBuffer;
          JsonObject& root = jsonBuffer.parseObject(json);

          // Test if parsing succeeds.
          if (!root.success()) {
            // parsing failed
            return ret; // bail out
          } else { // parsing successful, save file
            ret = root["cfgversion"];
            if (save==true) {
              File configFile = SPIFFS.open(cfgFileName, "w");
              if (!configFile) {
                ret = -1;
              }
              root.printTo(configFile);
              configFile.close();
            }
          }
      }
  } else {
      ret = -1;
  }

  http.end();
  return ret;
}


void fsConfig() { // load config json from FS
  if (safeMode) return; // bail out if we're in safemode
  fsVer = loadConfig(true); // try to load config from SPIFFS, set firmware version in memory
  /*if (fsVer!=-1) {
    sprintf(str, "version %d", fsVer);
  }*/
}

void getConfig() { // start the process to get config from api server
  if (safeMode) return; // bail out if we're in safemode
  // check with automation server to get latest config version number
  httpVer = requestConfig(false); // request config from server but don't update FS
  if (httpVer!=-1) { // automation server returned valid config version
    // compare FS config to server config - server config always wins
    if ((fsVer==-1) || (fsVer<httpVer)) { // failed to load config from FS or config outdated
      httpVer = requestConfig(true); // request config from server again, write to FS this time
      fsVer = loadConfig(false); // reload config from FS but don't reset firmware version
    }
  }
}

void handleCmd(char* cmdStr) { // handle commands from mqtt or websocket
  // using c string routines instead of Arduino String routines ... a lot faster
  char* cmdTxt = strtok(cmdStr, "=");
  char* cmdVal = strtok(NULL, "=");
  
  if (strcmp(cmdTxt, "marco")==0) setPolo = true; // respond to ping command
  else if (strcmp(cmdTxt, "reboot")==0) setReset = true; // reboot controller
  else if (strcmp(cmdTxt, "ch1en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        ch1en=1;
        digitalWrite(sw1, _ON); // nothing fancy for manual mode,
      } else { // OFF
        ch1en=0;
        digitalWrite(sw1, _OFF); // nothing fancy for manual mode,
      }
    }
  }
  else if (strcmp(cmdTxt, "ch2en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        ch2en=1;
        digitalWrite(sw2, _ON); // nothing fancy for manual mode,
      } else { // OFF
        ch2en=0;
        digitalWrite(sw2, _OFF); // nothing fancy for manual mode,
      }
    }
  }
  else if (strcmp(cmdTxt, "ch3en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        ch3en=1;
        digitalWrite(sw3, _ON); // nothing fancy for manual mode,
      } else { // OFF
        ch3en=0;
        digitalWrite(sw3, _OFF); // nothing fancy for manual mode,
      }
    }
  }
  else if (strcmp(cmdTxt, "ch4en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        ch4en=1;
        digitalWrite(sw4, _ON); // nothing fancy for manual mode,
      } else { // OFF
        ch4en=0;
        digitalWrite(sw4, _OFF); // nothing fancy for manual mode,
      }
    }
  }
  else if (strcmp(cmdTxt, "prttstat")==0) prtTstat = true; // print thermostat config
  else if (strcmp(cmdTxt, "update")==0) doUpdate = true; // upadte config from api
  else if (strcmp(cmdTxt, "getrgb")==0) getRGB = true; // print RGBW color values
  else if (strcmp(cmdTxt, "rgbtest")==0) rgbTest = true; // test RGBW output
  else if (strcmp(cmdTxt, "scani2c")==0) scanI2C = true; // print i2c bus scan
  else if (strcmp(cmdTxt, "gettime")==0) getTime = true; // print internal timestamp
  else if (strcmp(cmdTxt, "prtconfig")==0) prtConfig = true; // print running config
  else if (strcmp(cmdTxt, "prtlog")==0) prtLog = true; // print running config
  else if (strcmp(cmdTxt, "rmlog")==0) rmLog = true; // print running config
  else if (strcmp(cmdTxt, "rmconfig")==0) rmConfig = true; // print running config
}

// handle websocket events below
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
      case WStype_DISCONNECTED:
          //USE_SERIAL.printf("[%u] Disconnected!\n", num);
          wsConcount--;
          sprintf(str,"ws disconnect count=%d",wsConcount);
          mqttPrintStr(mqttpub, str);
          break;
      case WStype_CONNECTED:
          {
              IPAddress ip = webSocket.remoteIP(num);
              sprintf(str,"[%u] connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
              //USE_SERIAL.println();
              // send message to client
              sprintf(str,"Connection #%d.", num);
              webSocket.sendTXT(num, str);
              sprintf(str, "name=%s", nodename);
              webSocket.sendTXT(num, str);
              //webSocket.sendTXT(num, mqttpub);
              //webSocket.sendTXT(num, mqttsub);
              if (timeStatus() == timeSet) webSocket.sendTXT(num, "Time is set.");
              else webSocket.sendTXT(num, "Time not set.");
              //mqtt.publish(mqttpub, str);
              //wsSendlabels();
              newWScon = num + 1;
              wsConcount++;
              sprintf(str,"ws connect count=%u new=%u",wsConcount,newWScon);
              mqttPrintStr(mqttpub,str);
          }
          break;
      case WStype_TEXT:
          payload[length] = '\0'; // null terminate
          handleCmd((char *)payload);

          break;
      case WStype_BIN:
         // USE_SERIAL.printf("[%u] get binary lenght: %u\n", num, length);
          hexdump(payload, length);

          // send message to client
          // webSocket.sendBIN(num, payload, lenght);
          break;
  }
}

// receive mqtt messages
void mqttCallBack(char* topic, uint8_t* payload, unsigned int len) {
  char tmp[len];

  if (len<=0) return; // don't process null payload messages

  skipSleep=true; // don't go to sleep if we receive mqtt message
  strncpy(tmp, (char*)payload, len);
  tmp[len] = '\0';
  if (strstr(topic, "setspeed")) {
    if (tmp!=NULL) fanSpeed = atoi(tmp);}
  else if (strstr(topic, "setdir")) {
    if (tmp!=NULL) fanDirection = atoi(tmp);}
  else if (strstr(topic, "settemp")) {
    if (tmp!=NULL) tstatSet = atoi(tmp);}
  else if (strstr(topic, "setmode")) {
    if (tmp!=NULL) tstatMode = atoi(tmp);}
  else if (strstr(topic, "red")) {
    if (tmp!=NULL) red = atoi(tmp);}
  else if (strstr(topic, "green")) {
    if (tmp!=NULL) green = atoi(tmp);}
  else if (strstr(topic, "blue")) {
    if (tmp!=NULL) blue = atoi(tmp);}
  else if (strstr(topic, "white")) {
    if (tmp!=NULL) white = atoi(tmp);}
  else if (strstr(topic, "cmd")) {
    handleCmd(tmp);
  }
}

// maintain connection to mqtt broker
boolean mqttReconnect() {
  if (!useMQTT) return false; // bail out if mqtt is not configured
  char tmp[200];

  // Attempt to connect
  if (mqtt.connect(nodename)) {
    // Once connected, publish an announcement...
    mqttPrintStr(mqttpub, "Hello, world!");
    // ... and resubscribe
    mqtt.subscribe(mqttsub);
    prtConfig=true; // send out config report
    if (hasTstat) { // subscribe to thermostat topic
      sprintf(tmp, "%s/tstat/settemp\0", mqttbase);
      mqtt.subscribe(tmp);
      sprintf(tmp, "%s/tstat/setmode\0", mqttbase);
      mqtt.subscribe(tmp);
    }
    if (hasSpeed) { // subscribe to speed control topic
      sprintf(tmp, "%s/fan/setspeed\0", mqttbase);
      mqtt.subscribe(tmp);
      sprintf(tmp, "%s/fan/setdir\0", mqttbase);
      mqtt.subscribe(tmp);
    }
    if (hasRGB) { // subscrie to rgbw led topic
      sprintf(tmp, "%s/rgb/red\0", mqttbase);
      mqtt.subscribe(tmp);
      sprintf(tmp, "%s/rgb/green\0", mqttbase);
      mqtt.subscribe(tmp);
      sprintf(tmp, "%s/rgb/blue\0", mqttbase);
      mqtt.subscribe(tmp);
      sprintf(tmp, "%s/rgb/white\0", mqttbase);
      mqtt.subscribe(tmp);
    }
  }
  return mqtt.connected();
}

#ifdef _PWMC
void setupPWM() { // setup SDK pwm generator function
  	// Initial duty -> all off

	for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
		pwm_duty_init[channel] = 0;
	}
  
	// Period

	uint32_t period = PWM_PERIOD;

	// Initialize

	pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);

	// Commit

	pwm_start();
}
#endif

void setupADC() { // init routine for adc + io expander board
  if (!hasI2C) return; // bail out if no i2c

  //i2c_write(PUMP, IODIR, 0x00); // all ports output
  i2c_write(ADCPWR, IODIR, 0x01); // all ports except 0 output

  //i2c_write(PUMP, GPIO, 0x00); // all ports low
  i2c_write(ADCPWR, GPIO, 0x00); // all ports low

  ads.begin();
  ads.setGain(GAIN_ONE);
  ads.setSPS(ADS1115_DR_128SPS);

  // set flags for which channels are enabled
  if (sw1type==3) adcEnable=(adcEnable | 0x01);
  if (sw2type==3) adcEnable=(adcEnable | 0x02);
  if (sw3type==3) adcEnable=(adcEnable | 0x04);
  if (sw4type==3) adcEnable=(adcEnable | 0x08);
}

int16_t getAdc(uint8_t chan) {
  uint8_t gp = 1 << (chan + 1);
  int16_t result;

  i2c_write(ADCPWR, GPIO, gp); // power up port
  delay(5); // wait for stable
  result = ads.readADC_SingleEnded(chan); // take reading
  i2c_write(ADCPWR, GPIO, 0); // power down

  return result; // done
}

void printADC() { // print adc values to mqtt
  if (!useMQTT) return;
  for (int x=0; x<4; x++) {
    if (adcEnable&1<<x) {
      sprintf(str,"%s/adc%u",mqttbase,x); // example home/solar1/adc0
      mqttPrintInt(str, adcVal[x]);
    }
  }
}

void doADC() { // figure out which ADC to read and do it
  for (int x=0; x<4; x++) {
    if (adcEnable&1<<x) adcVal[x] = getAdc(x); // skip channels that are not enabled
  }
  printADC(); // display results
}

void setupOTA() { // init arduino ide ota library
  if (hasHostname) ArduinoOTA.setHostname(nodename);  // OTA hostname

  ArduinoOTA.onStart([]() {
    writeLog("ota", "start");
  });
  ArduinoOTA.onEnd([]() {
    writeLog("ota", "complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    writeLog("ota", "failure");

    //Serial.printf("Error[%u]: ", error);
    //if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    //else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    //else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    //else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    //else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin(); // start listening for arduinoota updates
}

void setupMQTT() {
  if (mqttport>0) { // port defined, setup connection
    useMQTT = true; // set a flag that mqtt is in use
    mqtt.setServer(mqttServer, mqttport); // setup mqtt broker connection
    mqtt.setCallback(mqttCallBack); // install function to handle incoming mqtt messages
  }
}

void checkMQTT() {
  if (!mqtt.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) lastReconnectAttempt = 0;
    }
  } else {
    // Client connected
    mqtt.loop();
  }
}

time_t getNtptime() {
  return ntpClient.getUnixTime();
}

void updateNTP() {
  getTime = false;
  time_t epoch = getNtptime();
  if (epoch < 1000000000) {
    mqttPrintStr("time", "Time not set, NTP unavailable.");
  } else {
    setTime(epoch); // set software rtc to current time
    mqttPrintStr("time", "Time set from NTP server.");
  }
}

void doSpeedout() {
    mqttPrintInt("fan/speed",fanSpeed);
    mqttPrintInt("fan/direction",fanDirection);

    wsSendInt("fanspd", fanSpeed);
    wsSendInt("fandir", fanDirection);
}

void wsData() { // send some websockets data if client is connected
  if (wsConcount<=0) return;

  // if (newWScon>0 && hasRGB) wsSwitchstatus(); // update switch status once for rgb controllers
  //else if (!hasRGB) wsSwitchstatus(); // regular updates for other node types
  wsSwitchstatus();

  if (timeStatus() == timeSet) wsSendTime("time=%d",now()); // send time to ws client

  if (hasRGB) return; // stop here if we're an rgb controller

  if (hasVout) { // send bat/vcc string
    wsSendFloat("volts", volts0);
    if (rawadc) wsSendInt("adc",adc0);
  }

  if (hasRSSI) wsSendInt("rssi",WiFi.RSSI()); // send rssi info

  if (hasTout) wsSendStr("temp",tmpChr); // send temperature

  if (hasIout) { // send readings from ADC
    wsSendFloat("amps0",amps0);
    if (rawadc) wsSendInt("raw0", raw0);

    wsSendFloat("amps1",amps1);
    if (rawadc) wsSendInt("raw1", raw1);

    wsSendFloat("volts", volts0);
    if (rawadc) wsSendInt("raw2", raw2);
  }
}

void mqttSendTime(time_t _time) {
  // if (hasRGB) return; // feature disabled if we're an rgb controller
  if ((!mqtt.connected()) || (!timeOut)) return; // bail out if there's no mqtt connection
  if (_time <= oldEpoch) return; // don't bother if it's been less than 1 second
  memset(str,0,sizeof(str));
  sprintf(str,"%ld", _time);
  mqttPrintStr("time", str);
  oldEpoch = _time;
}

void mqttData() { // send mqtt messages as required
  if (!mqtt.connected()) return; // bail out if there's no mqtt connection

  if (timeStatus() == timeSet) mqttSendTime(now());

  if (hasTout) mqttPrintStr("temp", tmpChr); // print temperature if equipped

  if (hasTstat) {
    mqttPrintInt("tstat/amb", tstatAmb);
    mqttPrintInt("tstat/rh", tstatRh); // print temperature if equipped
  }

  if (hasRGB) return; // further prints disabled if we're an rgb controller to save time

  if (hasVout) {
    mqttPrintFloat("volts", volts0); // print voltage if equipped
    if (rawadc) mqttPrintInt("volts/raw", adc0); // raw value
  }

  if (hasRSSI) mqttPrintInt("rssi", WiFi.RSSI()); // print RSSI

  if (hasIout) {
    mqttPrintFloat("amps/amps0", amps0);
    if (rawadc) mqttPrintInt("amps/raw0", raw0);

    mqttPrintFloat("amps/amps1", amps1);
    if (rawadc) mqttPrintInt("amps/raw1", raw1);

    mqttPrintFloat("volts", volts0);
    if (rawadc) mqttPrintInt("volts/raw", raw2);
  }
}

void doRGB() { // send updated values to the first four channels of the pwm chip
  // need to expand this to support four 4-channel groups, some sort of array probably
  uint8_t _r,_g,_b,_w;
  _r = (rgbwChan>>6)&3; // first nibble
  _g = (rgbwChan>>4)&3; // next
  _b = (rgbwChan>>2)&3; // third
  _w = (rgbwChan&3); // fourth, mask off the 6 most significant bits

  if (oldred!=red) {
    sprintf(str,"Update red from %u to %u", oldred,red);
    mqttPrintStr(mqttpub, str);
    oldred=red;
    rgbw.setpwm(_r, red);
  }

  if (oldgreen!=green) {
    sprintf(str,"Update green from %u to %u", oldgreen,green);
    mqttPrintStr(mqttpub, str);
    oldgreen=green;
    rgbw.setpwm(_g, green);
  }

  if (oldblue!=blue) {
    sprintf(str,"Update blue from %u to %u", oldblue,blue);
    mqttPrintStr(mqttpub, str);
    oldblue=blue;
    rgbw.setpwm(_b, blue);
  }

  if (oldwhite!=white) {
    sprintf(str,"Update white from %u to %u", oldwhite,white);
    mqttPrintStr(mqttpub, str);
    oldwhite=white;
    rgbw.setpwm(_w, white);
  }
}


void testRGB() {
  red=255; blue=0; green=0; white=0;
  doRGB();
  delay(250);
  red=0; blue=255; green=0; white=0;
  doRGB();
  delay(250);
  red=0; blue=0; green=255; white=0;
  doRGB();
  delay(250);
  red=0; blue=0; green=0; white=255;
  doRGB();
  delay(250);
  red=0; blue=0; green=0; white=0;
  doRGB();
  rgbTest = false;
}

void setupRGB() { // init pca9633 pwm chip
  // pwm.begin(); // default address is 40
  // pwm.setPWMFreq(200);  // This is the maximum PWM frequency
  rgbw.begin(0x62); // TODO this should be set from config?

  // set all 4 channels off, just for kicks
  rgbw.setrgbw(0,0,0,0);

  uint8_t _r,_g,_b,_w;
  _r = (rgbwChan>>6)&3; // first nibble
  _g = (rgbwChan>>4)&3; // next
  _b = (rgbwChan>>2)&3; // third
  _w = (rgbwChan&3); // fourth, mask off the 6 most significant bits

  sprintf(str,"RGBW channel assignments %u %u %u %u", _r, _g, _b, _w);
  mqttPrintStr(mqttpub, str);

}

void setupADS() {
  ads.begin();
  ads.setGain(GAIN_ONE);
  ads.setSPS(ADS1115_DR_64SPS);
}

void setupSpeed() {
  speedAddr = sw1type;
  sprintf(str,"Fan speed function using i2c device %u.", speedAddr);
  mqttPrintStr(mqttpub, str);
  if (coldBoot) {
    speedControl(0,0); // direction 0, speed 0
  }
}

void setupTstat() {
  dht.setup(DHTPIN, DHTesp::DHT11);
  sprintf(str,"Thermostat function using DHT on GPIO %u.", DHTPIN);
  mqttPrintStr(mqttpub, str);

  if (coldBoot) { // latching relay power-on state is unknowable, so make sure it's set to off
    digitalWrite(sw1, _OFF);
    delay(100);
    digitalWrite(sw2, _ON);
    delay(100);
    digitalWrite(sw2, _OFF);
    mqttPrintStr(mqttpub, "Thermostat re-initialized.");
  }
}

void printConfig() { // print relevant config bits out to mqtt
  printReboot();
  if (timeOut) mqttPrintStr(mqttpub, "Timestamp enabled.");
  if (hasFan) mqttPrintStr(mqttpub, "Fan control enabled.");
  if (hasRGB) mqttPrintStr(mqttpub, "RGBW control enabled.");
  if (hasRSSI) mqttPrintStr(mqttpub, "RSSI reporting enabled.");
  if (hasVout) mqttPrintStr(mqttpub, "Voltage reporting enabled.");
  if (hasTout) mqttPrintStr(mqttpub, "Temperature reporting enabled.");
  if (hasTstat) mqttPrintStr(mqttpub, "Thermostat function enabled.");
  printIOTaddr();
  printIOTurl();
  printMQTTaddr();
  printNTPaddr();
  printSwitches();
  prtConfig=false;
}

void doTout() {
  String vStr;

  memset(tmpChr,0,sizeof(tmpChr));
  if (hasTpwr>0) {
    digitalWrite(hasTpwr, HIGH); // ow on
    delay(5); // wait for powerup
  }

  ds18b20.requestTemperatures();
  byte retry = 20;
  float temp=0.0;
  do {
    temp = ds18b20.getTempCByIndex(0);
    retry--;
    delay(2);
  } while (retry > 0 && (temp == 85.0 || temp == (-127.0)));

  if (hasTpwr>0) {
    digitalWrite(hasTpwr, LOW); // ow off
  }


  vStr = String(temp,3);
  vStr.toCharArray(tmpChr, vStr.length()+1);
}

void doTstat() { // perform thermostat functions
  // read data from DHT11 module

  float h = dht.getHumidity();
  float c = dht.getTemperature();

  tstatAmb = (int) c;
  tstatRh = (uint8_t) h;

  // super simple hystersis logic
  // maybe one day add some code to take weather condition, forecast and ambient outdoor temp into regard
  if (tstatMode==1) { // heating Mode
    if (tstatAmb<(tstatSet-1)) { // one degree below setpoint, pulse relay on briefly
      digitalWrite(sw1, _ON);
      delay(100);
      digitalWrite(sw1, _OFF);
      tstatOper=2; // tell everyone relay is on
    }

    if (tstatAmb>(tstatSet+1)) { // one degree above setpoint, pulse relay off briefly
      digitalWrite(sw2, _ON);
      delay(100);
      digitalWrite(sw2, _OFF);
      tstatOper=1; // relay off
    }
  } else if (tstatMode==2) { // cooling mode
    if (tstatAmb<(tstatSet-1)) { // one degree below setpoint, pulse relay off briefly
      digitalWrite(sw2, _ON);
      delay(100);
      digitalWrite(sw2, _OFF);
      tstatOper=1;
    }

    if (tstatAmb>(tstatSet+1)) { // one degree above setpoint, pulse relay on briefly
      digitalWrite(sw1, _ON);
      delay(100);
      digitalWrite(sw1, _OFF);
      tstatOper=2;
    }
  } else {
    tstatOper=0; // thermostat disabled
  }
  mqttPrintInt("tstat/oper", tstatOper);
}

void printTstat() { // print thermostat configuration specifics
  prtTstat = false;
  if (hasTstat) {
    mqttPrintStr(mqttpub, "Sending thermostat report.");
    sprintf(str,"Thermostat function using DHT on GPIO %u\0.", DHTPIN);
    mqttPrintStr(mqttpub, str);

    doTstat(); // update data

    const char* _status = dht.getStatusString();

    mqttPrintStr("tstat/dht", (char*) _status);
  	mqttPrintInt("tstat/mode", tstatMode);
    mqttPrintInt("tstat/set", tstatSet);
    mqttPrintInt("tstat/amb", tstatAmb);
    mqttPrintInt("tstat/rh", tstatRh);
  } else {
    mqttPrintStr(mqttpub, "Thermostat function disabled.");
  }

}

void setup() {
  memset(tmpChr,0,sizeof(tmpChr));

    // if the program crashed, skip things that might make it crash
  String rebootMsg = ESP.getResetReason();
  if (rebootMsg=="Exception") safeMode=true;
  else if (rebootMsg=="Hardware Watchdog") safeMode=true;
  else if (rebootMsg=="Unknown") safeMode=true;
  else if (rebootMsg=="Software Watchdog") safeMode=true;
  else if (rebootMsg=="Deep-Sleep Wake") coldBoot=false;

  // record reboot reason to log
  rebootMsg.toCharArray(rebootChar, rebootMsg.length()+1);
  writeLog("reboot",rebootChar);

  // "mount" the filesystem
  bool success = SPIFFS.begin();
  if (!success) SPIFFS.format();

  if (!safeMode) fsConfig(); // read node config from FS

#ifdef _TRAILER
  WiFi.begin("DXtrailer", "2317239216");
#else
  WiFi.begin("Tell my WiFi I love her", "2317239216");
#endif

  int wifiConnect = 90;
  while ((WiFi.status() != WL_CONNECTED) && (wifiConnect-- > 0)) { // spend 90 seconds trying to connect to wifi
    // connecting to wifi
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED ) { // still not connected? reboot!
    writeLog("reboot","no wifi at boot");
    ESP.reset();
    delay(5000);
  }

  if (hasHostname) { // valid config found on FS, set network name
    WiFi.hostname(String(nodename)); // set network hostname
    MDNS.begin(nodename); // set mDNS hostname
  }

  WiFi.macAddress(mac); // get esp mac address, store it in memory, build fw update url
  sprintf(macStr,"%x%x%x%x%x%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf(theURL,"/iotfw?mac=%s", macStr);

  // request latest config from web api
  if (!safeMode) getConfig();

  // check web api for new firmware
  if (!safeMode) httpUpdater();

  // test ntp connection
  updateNTP();

  setSyncProvider(getNtptime); // use NTP to get current time
  setSyncInterval(600); // refresh clock every 10 min

  // start websockets server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // setup other things
  setupOTA();
  setupMQTT();

  if (useMQTT) {
    // String rebootReason = String("Last reboot cause was ") + rebootMsg;
    // rebootReason.toCharArray(str, rebootReason.length()+1);
    rebootMsg.toCharArray(str, rebootMsg.length()+1);
    mqttPrintStr("reboot/reason", str);
  }

  // setup i2c if configured, basic sanity checking on configuration
  if (hasI2C && iotSDA>=0 && iotSCL>=0 && iotSDA!=iotSCL) {
    sprintf(str,"I2C enabled SDA=%u SCL=%u", iotSDA, iotSCL);
    mqttPrintStr(mqttpub, str);

    Wire.begin(iotSDA, iotSCL); // from api config file

    //Wire.begin(12, 14); // from api config file
    i2c_scan();
  }

  // setup any connected modules
  if (hasRGB) setupRGB();
  if (hasIout) setupADS();
  if (hasSpeed) setupSpeed();
  if (hasADC) setupADC();
  if (hasTstat) setupTstat();

  if (OWDAT>0) { // setup onewire if data line is using pin 1 or greater
    sprintf(str,"Onewire data GPIO %u", OWDAT);
    mqttPrintStr(mqttpub, str);
    oneWire.begin(OWDAT);
    if (hasTout) {
      ds18b20 = DallasTemperature(&oneWire);
      ds18b20.begin(); // start one wire temp probe
    }
    if (hasTpwr>0) {
      sprintf(str,"Onewire power GPIO %u", hasTpwr);
      mqttPrintStr(mqttpub, str);
      pinMode(hasTpwr, OUTPUT); // onewire power pin as output
      digitalWrite(hasTpwr, LOW); // ow off
    }
  }

  writeLog("system", "online");
} // end of setup()

void doVout() {
  int vBat=vccOffset;
  float voltage=0.00;
  //String vStr;
  //memset(voltsChr,0,sizeof(voltsChr));
  //memset(adcChr,0,sizeof(adcChr));

  if (useGetvcc) {
    vBat += ESP.getVcc(); // internal voltage reference (Vcc);
    voltage = vBat / 1000.0;
    //vStr = String(voltage,3);
  } else {
    vBat += analogRead(A0); // read the TOUT pin
    voltage = vBat * (vccDivisor / 1023.0); // adjust value, set 5.545 equal to your maximum expected input voltage
    //vStr = String(voltage,3);
  }
  volts0 = voltage;
  adc0 = vBat;
  //sprintf(adcChr, "adc=%d", vBat);
  //vStr.toCharArray(voltsChr, vStr.length()+1);
}

void doRGBout() {
  if(hasRGB) {
  	memset(str,0,sizeof(str));
  	sprintf(str, "red=%u,green=%u,blue=%u,white=%u", red, green, blue, white);
  	mqttPrintStr(mqttpub, str);
  	wsSend(str);
  }
  getRGB=false;
}

void doRSSI() {
  // int rssi = WiFi.RSSI();
  // memset(rssiChr,0,sizeof(rssiChr));
  // sprintf(rssiChr, "%d", rssi);
}

void doSpeed() {
  speedControl(fanSpeed, fanDirection);
  doSpeedout();
}

void doIout() { // enable current reporting if module is so equipped
  if (hasRGB) return; // feature disabled if we're an rgb controller
  int16_t adc0, adc1, adc2;
  if (!hasI2C) return;

  adc0 = ads.readADC_SingleEnded(1) ; // adc channel 1 = switch 0 (switch one) ** this should probably be programmable, based on current PCB design
  raw0 = adc0;
  float voltage0 = (adc0 / 32767.0) * 4096.0;
  amps0 = (voltage0 - ACSoffset) / mvPerA; // 44.0 = mv per amp from datasheet

  adc1 = ads.readADC_SingleEnded(0) ; // adc channel 0 = switch 1 (switch two)
  raw1 = adc1;
  float voltage1 = (adc1 / 32767.0) * 4096.0;
  amps1 = (voltage1 - ACSoffset) / mvPerA; // 44.0 = mv per amp from datasheet

  adc2 = ads.readADC_SingleEnded(2) ; // adc channel 2 = battery voltage divider
  raw2 = adc2;
  float voltage2 = 0.0;
  if (altAdcvbat) voltage2 = (adc2 + vccOffset) / 1000.0;  // funky method for original current switch
  else voltage2 = (adc2 / 32767.0) * vccDivisor;  // proper method, 6.8k / 2.2k voltage divider

  if (amps0<0.010) amps0=0.0; // less than 10ma, might be an error disgregard
  if (amps1<0.010) amps1=0.0;
  if (voltage2<0.1) voltage2=0.0;

  volts0 = voltage2;
}

void runUpdate() { // test for http update flag, received url via mqtt
  doUpdate = false; // clear flag
  updateCnt = 0; // clear update counter
  if (useMQTT && !hasRGB) {
    mqttPrintStr(mqttpub, "Checking for updates");
    mqtt.loop();
  }
  delay(50);
  getConfig();
  httpUpdater();
}

void doPolo() {
      setPolo = false; // respond to an mqtt 'ping' of sorts
      mqttPrintStr(mqttpub, "Polo");
}

void loop() {
  long loopNow = millis();

  if (safeMode) { // safeMode engaged, enter blocking loop wait for an OTA update
    int safeDelay=30000; // five minutes in 100ms counts
    while (safeDelay--) {
      ArduinoOTA.handle();
      delay(100);
    }
    writeLog("reboot","safemode");
    ESP.reset(); // restart, try again
    delay(5000); // give esp time to reboot
  }
  
  ArduinoOTA.handle(); // handle OTA updates
  if (useMQTT) checkMQTT(); // keep mqtt alive if enabled

  webSocket.loop(); // keep websocket alive if enabled

  if (hasRGB) doRGB(); // rgb updates
  if (setReset) doReset(); // execute reboot command

  if (loopNow - loopTimer > updateRate*10) { // code below runs every few seconds
    loopTimer = loopNow;
    mqttPrintInt("uptime",loopNow/1000);

    if (hasADC)   doADC();
    if (hasTstat) doTstat();
    if (hasRSSI)  doRSSI();
    if (hasTout)  doTout();
    if (hasVout)  doVout();
    if (hasIout)  doIout();
    if (hasSpeed) doSpeed();

    if (rgbTest)  testRGB(); // respond to testrgb command
    if (getRGB)   doRGBout();

    if (setPolo)  doPolo(); // respond to ping

    if (updateCnt++ > 6) {
      updateCnt=0;
      runUpdate(); // check for config update 
    }

    if (wsConcount>0) wsData();
    if (useMQTT) mqttData(); // regular update for non RGB controllers

    if (prtConfig) printConfig(); // config print was requested

    if (prtLog) readLog(); // log dump requested
    if (rmLog) deleteLog(); // Remove log file

    if (rmConfig) deleteConfig(); // Remove IOT config file
    if (prtTstat) printTstat(); // update thermostat if commanded

    if (getTime) updateNTP(); // update time if requested by command

    if (scanI2C) i2c_scan(); // respond to i2c scan command

    /*if ((!skipSleep) && (sleepEn)) {
      sprintf(str,"Sleeping in %u seconds.", (updateRate*20/1000));
      mqttPrintStr(mqttpub, str);
    }
    */
  }


  /*
  if ((!skipSleep) && (sleepEn)) {
    if ((sleepPeriod<30) || (sleepPeriod>4294)) sleepPeriod=900; // prevent sleeping for less than 1 minute or more than the counter will allow, roughly 1 hour
    sprintf(myChr,"Back in %d seconds.", sleepPeriod);
    if (useMQTT) {
      mqttPrintStr(mqttpub, myChr);
      mqtt.loop();
    }

    ESP.deepSleep(1000000 * sleepPeriod, WAKE_RF_DEFAULT); // sleep for 15 min
    delay(5000); // give esp time to fall asleep

  }

  skipSleep = false;
  */
}
