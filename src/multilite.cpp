#include <Arduino.h>
#include <ArduinoJson.h>
#include "OneWire.h"
#include <Time.h>
#include <FS.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
//#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "DallasTemperature.h"
#include "pca9633.h"
#include "Adafruit_ADS1015.h"
#include <Wire.h>
#include "wifiap.h" // local wifi information

// uncomment for ac switch module, leave comment for dc switch module
//#define _ACMULTI true
//#define _TRAILER true
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
const char* iotSrv = "192.168.10.30"; // automation api server name
#else // iot node for home application
const char* iotSrv = "192.168.2.30"; // automation api server name
#endif
const char* jsonFile = "/iot.json"; // fs config filename
const int jsonSize = 1024;

char foo[8];
char rssiChr[10];
char myChr[32];
char voltsChr[10];
char amps0Chr[10];
char amps1Chr[10];
char adcChr[10];
unsigned char wsConcount=0;
int raw0=0, raw1=0, raw2=0;
char tmpChr[10];
unsigned char mac[6];
char macStr[12];
char url[100];
char str[60];
char sw1label[32], sw2label[32], sw3label[32], sw4label[32];
char nodename[32];
char mqttserver[32];
char vdivsor[8];
int OWDAT=-1; //
int  mqttport=0;
char mqttpub[100], mqttsub[100];
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
int updateRate = 30;
time_t oldEpoch = 0;
uint8_t red=0,green=0,blue=0,white=0;
unsigned char updateCnt = 0;
unsigned char newWScon = 0;
unsigned char mqttFail = 0;
unsigned char speedAddr = 0; // i2c address for speed control chip
unsigned char fanSpeed=0, fanDirection=0;
int sw1 = -1, sw2 = -1, sw3 = -1, sw4 = -1;
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
bool skipSleep = false; // skip next sleep cycle
bool sleepEn = false; // disable sleep entirely
bool useGetvcc = false; // use internal divider network
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
bool doReset = false; // flag for reboot
bool hasHostname = false; // flag for hostname being set by saved config
bool scanI2C = false;
bool rgbTest = false;
bool prtConfig = false; // flag to request config print via mqtt
unsigned char ntpOffset = 4; // offset from GMT
uint8_t iotSDA = 12, iotSCL = 14; // i2c bus pins

int ch1en = -1, ch2en = -1, ch3en = -1, ch4en = -1;

Adafruit_ADS1115 ads;
WiFiClient espClient;
PubSubClient mqtt(espClient);
OneWire oneWire;
DallasTemperature ds18b20 = NULL;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WiFiMulti wifiMulti;
PCA9633 rgbw; // instance of pca9633 library

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
IPAddress timeServerIP; // time.nist.gov NTP server address
unsigned int localPort = 2390;      // local port to listen for UDP packets
const char* ntpServerName = "us.pool.ntp.org";
const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
uint8_t packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

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

char* cleanStr(const char* _str) {
  int x=0, i=0;
  char c;
  memset(str,0,sizeof(str)); // zero out array

  while (((c = _str[i++]) != '\0') && (x<59)) { // read array until we hit a null
    if (isPrintable(c)) str[x++] = c; // exclude character that are not alphaNumeric
  }
  str[x] = '\0'; // null terminate

  return str; // return printable results
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

  if (useMQTT) mqtt.publish(mqttpub, "Scanning I2C Bus...");

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
        mqtt.publish(mqttpub, str);
        mqtt.loop();
      }
      delay(10);
      nDevices++;
    }
  }
  if (useMQTT) mqtt.publish(mqttpub, "I2C scan complete.");
}

void httpUpdater() {
  t_httpUpdate_return ret = ESPhttpUpdate.update(iotSrv, iotPort, theURL, fwversion);

  switch(ret) {
      case HTTP_UPDATE_FAILED:
        if (useMQTT) mqtt.publish(mqttpub, "FW update failed");
        delay(10);
        break;

      case HTTP_UPDATE_NO_UPDATES:
        if (useMQTT && !hasRGB) mqtt.publish(mqttpub, "No FW update available");
        delay(10);
        break;

      case HTTP_UPDATE_OK:
        break;
  }
}

void wsSendTime(const char* msg, time_t mytime) {
  memset(str,0,sizeof(str));
  sprintf(str, msg, mytime);
  wsSend(str);
}

void speedControl(uint8_t fanSpeed, uint8_t fanDirection) {
  if (!hasI2C) return; // bail out if i2c not setup

  sprintf(str,"Fan %u dir %u speed %u", speedAddr, fanDirection, fanSpeed);
  if (useMQTT) mqtt.publish(mqttpub, str);

  if (speedAddr>0) {
    i2c_write(speedAddr, 0x03, fanDirection);
    delay(10);
    i2c_write(speedAddr, 0x00, fanSpeed);
  }
}

int loadConfig(bool setFSver) {
  int ver = -1;
  File configFile = SPIFFS.open(jsonFile, "r");
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
  if (json.containsKey("sw1label"))   strcpy(sw1label, json["sw1label"]);
  if (json.containsKey("sw2label"))   strcpy(sw2label, json["sw2label"]);
  if (json.containsKey("sw3label"))   strcpy(sw3label, json["sw3label"]);
  if (json.containsKey("sw4label"))   strcpy(sw4label, json["sw4label"]);
  if (json.containsKey("mqttserver")) strcpy(mqttserver, json["mqttserver"]);
  if (json.containsKey("vccdivsor"))  vccDivisor = atof((const char*)json["vccdivsor"]);
  if (json.containsKey("mvpera")) mvPerA = atof((const char*)json["mvpera"]);


  if (json.containsKey("mqttpub")) {
    const char* mbase = json["mqttbase"];
    const char* mpub = json["mqttpub"];
    const char* msub = json["mqttsub"];
    sprintf(mqttpub, "%s/%s/%s",mbase, nodename, mpub);
    sprintf(mqttsub, "%s/%s/%s",mbase, nodename, msub);
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
  hasTpwr = json["hastpwr"]; // also serves as OWPWR
  OWDAT = json["owdat"]; // set onewire data pin
  hasI2C = json["hasi2c"];
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
  //hasDimmer = json["hasdimmer"];

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

void wsSendlabels(uint8_t _num) { // send switch labels only to newly connected websocket client
  // uint8_t _num = _x - 1; // client number is one less
  // if (useMQTT) mqtt.publish(mqttpub, "wsSendlabels");
  // if (_num == 0) return;
  memset(str,0,sizeof(str));
  sprintf(str,"sending labels: sw1=%d %d sw2=%d %d sw3=%d %d sw4=%d %d",sw1,sw1type,sw2,sw2type,sw3,sw3type,sw4,sw4type);
  if (useMQTT) mqtt.publish(mqttpub, str);
  wsSend(str);
  char labelStr[8];
  if (sw1>=0) {
    if (sw1type==0) strcpy(labelStr,"switch\0");
    else if (sw1type==1) strcpy(labelStr,"label\0");
    else if (sw1type==2) strcpy(labelStr,"rgb\0");
    else if (sw1type>=8) strcpy(labelStr,"fan\0");
    sprintf(str,"%s=%s",labelStr, sw1label);
    wsSend(str);
  }
  if (sw2>=0) {
    if (sw2type==0) strcpy(labelStr,"switch\0");
    else if (sw2type==1) strcpy(labelStr,"label\0");
    else if (sw2type==2) strcpy(labelStr,"rgb\0");
    sprintf(str,"%s=%s",labelStr, sw2label);
    wsSend(str);
  }
  if (sw3>=0) {
    if (sw3type==0) strcpy(labelStr,"switch\0");
    else if (sw3type==1) strcpy(labelStr,"label\0");
    else if (sw3type==2) strcpy(labelStr,"rgb\0");
    sprintf(str,"%s=%s",labelStr, sw3label);
    wsSend(str);
  }
  if (sw4>=0) {
    if (sw4type==0) strcpy(labelStr,"switch\0");
    else if (sw4type==1) strcpy(labelStr,"label\0");
    else if (sw4type==2) strcpy(labelStr,"rgb\0");
    sprintf(str,"%s=%s",labelStr, sw4label);
    wsSend(str);
  }
  // i2c_scan();
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
              File configFile = SPIFFS.open(jsonFile, "w");
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

void handleMsg(char* cmdStr) { // handle commands from mqtt or websocket
  // using c string routines instead of Arduino String routines ... a lot faster
  char* cmdTxt = strtok(cmdStr, "=");
  char* cmdVal = strtok(NULL, "=");

  if (strcmp(cmdTxt, "marco")==0) setPolo = true;
  else if (strcmp(cmdTxt, "update")==0) doUpdate = true;
  else if (strcmp(cmdTxt, "getrgb")==0) getRGB = true;
  else if (strcmp(cmdTxt, "rgbtest")==0) rgbTest = true;
  else if (strcmp(cmdTxt, "scani2c")==0) scanI2C = true;
  else if (strcmp(cmdTxt, "reboot")==0) doReset = true;
  else if (strcmp(cmdTxt, "gettime")==0) getTime = true;
  else if (strcmp(cmdTxt, "prtconfig")==0) prtConfig = true;
  else if (strcmp(cmdTxt, "fanspd")==0) fanSpeed = atoi(cmdVal);
  else if (strcmp(cmdTxt, "fandir")==0) fanDirection = atoi(cmdVal);
  else if (strcmp(cmdTxt, "red")==0) red = atoi(cmdVal);
  else if (strcmp(cmdTxt, "green")==0) green = atoi(cmdVal);
  else if (strcmp(cmdTxt, "blue")==0) blue = atoi(cmdVal);
  else if (strcmp(cmdTxt, "white")==0) white = atoi(cmdVal);
  else {
    int i = atoi(cmdVal);
    if (strcmp(cmdTxt, "ch1en")==0) {
      if (i == 1) { // ON
        ch1en=1;
        digitalWrite(sw1, _ON); // nothing fancy for manual mode, 
      } else { // OFF
        ch1en=0;
        digitalWrite(sw1, _OFF); // nothing fancy for manual mode, 
      }
    }
    else if (strcmp(cmdTxt, "ch2en")==0) {
      if (i == 1) { // ON
        ch2en=1;
        digitalWrite(sw2, _ON); // nothing fancy for manual mode, 
      } else { // OFF
        ch2en=0;
        digitalWrite(sw2, _OFF); // nothing fancy for manual mode, 
      }
    }

  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
      case WStype_DISCONNECTED:
          //USE_SERIAL.printf("[%u] Disconnected!\n", num);
          wsConcount--;
          sprintf(str,"ws disconnect count=%d",wsConcount);
          if (useMQTT) mqtt.publish(mqttpub,str);
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
              if (useMQTT) mqtt.publish(mqttpub,str);
          }
          break;
      case WStype_TEXT:
          payload[length] = '\0'; // null terminate
          handleMsg((char *)payload);

          break;
      case WStype_BIN:
         // USE_SERIAL.printf("[%u] get binary lenght: %u\n", num, length);
          hexdump(payload, length);

          // send message to client
          // webSocket.sendBIN(num, payload, lenght);
          break;
  }
}


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

time_t getNtptime() {
  time_t epoch = 0;

  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(500);

  int cb = udp.parsePacket();
  if (!cb) {
    //Serial.println("no packet!?");
    wsSend("NTP Error");
  }
  else {
    // Serial.print("packet received, length=");
    // Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // Serial.print("Seconds since Jan 1 1900 = " );
    // Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    // Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

    epoch = epoch - (60 * 60 * ntpOffset); // take off 4 hrs for EDT offset
    sprintf(str, "NTP epoch=%d", epoch);
    wsSend(str);
  }
  return epoch;
}

void mqttcallback(char* topic, byte* payload, unsigned int len) {
  skipSleep=true; // don't go to sleep if we receive mqtt message
  char tmp[200];
  strncpy(tmp, (char*)payload, len);
  tmp[len] = 0x00;
  handleMsg(tmp);
}

void mqttreconnect() {
  // Loop until we're reconnected
  if (!useMQTT) return; // bail out if mqtt is not configured
  int retry = 0;
  if (mqttFail>=100) { // repeated mqtt failure could mean network trouble, reboot esp
    ESP.reset();
    delay(5000); // give esp time to reset
  }
  while (!mqtt.connected()) {
    // Attempt to connect
    if (mqtt.connect(nodename)) {
      // Once connected, publish an announcement...
      mqtt.publish(mqttpub, "Hello, world!");
      //mqtt.publish(mqttpub, macStr);
      // ... and resubscribe
      mqtt.subscribe(mqttsub);
    } else {
      // Wait before retrying
      delay(100);
    }
    if (retry++ > 4) {
      mqttFail++;
      return; // bail out after 5 attempts
    }
  }
}

void setupOTA() { // init arduino ide ota library
  ArduinoOTA.onStart([]() {
    //Serial.print("OTA Update");
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("done!");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
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
    mqtt.setServer(mqttserver, mqttport); // setup mqtt broker connection
    mqtt.setCallback(mqttcallback); // install function to handle incoming mqtt messages
    mqttreconnect(); // check mqqt status
  }
}

void updateNTP() {
  getTime = false;
  time_t epoch = getNtptime();
  if (epoch == 0) {
    if (useMQTT) mqtt.publish(mqttpub, "Time not set, NTP unavailable.");
  } else {
    setTime(epoch); // set software rtc to current time
    if (useMQTT) mqtt.publish(mqttpub, "Time set from NTP server.");
  }
}

void doSpeedout() {
    sprintf(str,"fanspd=%u", fanSpeed);
    if (useMQTT) mqtt.publish(mqttpub, str);
    wsSend(str);
    sprintf(str,"fandir=%u", fanDirection);
    if (useMQTT) mqtt.publish(mqttpub, str);
    wsSend(str);
}

void wsData() { // send some websockets data if client is connected
  if (wsConcount<=0) return;

  // if (newWScon>0 && hasRGB) wsSwitchstatus(); // update switch status once for rgb controllers
  //else if (!hasRGB) wsSwitchstatus(); // regular updates for other node types
  wsSwitchstatus();

  if (timeStatus() == timeSet) wsSendTime("time=%d",now()); // send time to ws client

  if (hasRGB) return; // stop here if we're an rgb controller

  if (hasVout) { // send bat/vcc string
    wsSend(voltsChr);
    if (rawadc) wsSend(adcChr);
  }

  if (hasRSSI) wsSend(rssiChr); // send rssi info
  if (hasSpeed) doSpeedout();

  if (hasTout) wsSend(tmpChr); // send temperature

  if (hasIout) { // send readings from ADC
    sprintf(str,"raw0=%d", raw0);
    wsSend(amps0Chr);
    if (rawadc) wsSend(str);
    memset(str,0,sizeof(str));
    sprintf(str,"raw1=%d", raw1);
    wsSend(amps1Chr);
    if (rawadc) wsSend(str);
    memset(str,0,sizeof(str));
    sprintf(str,"raw2=%d", raw2);
    wsSend(voltsChr);
    if (rawadc) wsSend(str);
    memset(str,0,sizeof(str));
  }
}

void mqttSendTime(time_t _time) {
  // if (hasRGB) return; // feature disabled if we're an rgb controller
  if (!mqtt.connected()) return; // bail out if there's no mqtt connection
  if (_time <= oldEpoch) return; // don't bother if it's been less than 1 second
  memset(str,0,sizeof(str));
  sprintf(str,"time=%d", _time);
  mqtt.publish(mqttpub, str);
  oldEpoch = _time;
}

void mqttData() { // send mqtt messages as required
  if (!mqtt.connected()) return; // bail out if there's no mqtt connection

  if (timeStatus() == timeSet) mqttSendTime(now());

  if (hasRGB) return; // feature disabled if we're an rgb controller

  if (hasTout) mqtt.publish(mqttpub, tmpChr);


  if (hasVout) {
    mqtt.publish(mqttpub, voltsChr);
    if (rawadc) mqtt.publish(mqttpub, adcChr);
  }

  if (hasRSSI) mqtt.publish(mqttpub, rssiChr);

  if (hasIout) {
    mqtt.publish(mqttpub, amps0Chr);
    sprintf(str,"raw0=%d", raw0);
    if (rawadc) mqtt.publish(mqttpub, str);

    mqtt.publish(mqttpub, amps1Chr);
    sprintf(str,"raw1=%d", raw1);
    if (rawadc) mqtt.publish(mqttpub, str);

    mqtt.publish(mqttpub, voltsChr);
    sprintf(str,"raw2=%d", raw2);
    if (rawadc) mqtt.publish(mqttpub, str);
  }

  if (hasVout) {
    mqtt.publish(mqttpub, voltsChr);
    if (rawadc) mqtt.publish(mqttpub, adcChr);
  }

  if (hasRSSI) mqtt.publish(mqttpub, rssiChr);
  if (hasSpeed) doSpeedout();
}


void doRGB() { // send updated values to the first four channels of the pwm chip
  // need to expand this to support four 4-channel groups, some sort of array probably
  rgbw.setpwm(0, red);
  rgbw.setpwm(1, green);
  rgbw.setpwm(2, blue);
  rgbw.setpwm(3, white);
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

}

void setupADS() {
  ads.begin();
  ads.setGain(GAIN_ONE);
  ads.setSPS(ADS1115_DR_64SPS);
}

void setupSpeed() {
  speedAddr = sw1type;
  sprintf(str,"Fan speed control enabled, using device %u.", speedAddr);
  mqtt.publish(mqttpub, str);
  speedControl(0,0); // direction 0, speed 0
}

void printConfig() { // print relevant config bits out to mqtt
  if (hasFan) mqtt.publish(mqttpub, "Fan control enabled.");
  if (hasRGB) mqtt.publish(mqttpub, "RGBW control enabled.");
  if (hasRSSI) mqtt.publish(mqttpub, "RSSI reporting enabled.");
  if (hasVout) mqtt.publish(mqttpub, "Voltage reporting enabled.");
  if (hasTout) mqtt.publish(mqttpub, "Temperature reporting enabled.");
  prtConfig=false;
}

void setup() {
  memset(voltsChr,0,sizeof(voltsChr));
  memset(amps0Chr,0,sizeof(amps0Chr));
  memset(amps1Chr,0,sizeof(amps1Chr));
  memset(tmpChr,0,sizeof(tmpChr));

    // if the program crashed, skip things that might make it crash
  String rebootMsg = ESP.getResetReason();
  if (rebootMsg=="Exception") safeMode=true;
  else if (rebootMsg=="Hardware Watchdog") safeMode=true;
  else if (rebootMsg=="Unknown") safeMode=true;
  else if (rebootMsg=="Software Watchdog") safeMode=true;

  if (sw1>=0) {
    pinMode(sw1, OUTPUT);
  }
  if (sw2>=0) {
    pinMode(sw2, OUTPUT);
  }
  if (sw3>=0) {
    pinMode(sw3, OUTPUT);
  }
  if (sw4>=0) {
    pinMode(sw4, OUTPUT);
  }

  // "mount" the filesystem
  bool success = SPIFFS.begin();
  if (!success) SPIFFS.format();

  if (!safeMode) fsConfig(); // read node config from FS

#ifdef _TRAILER
  wifiMulti.addAP(_GLM_WIFI_AP2,_GLM_WIFI_PW2);
#else
  wifiMulti.addAP(_GLM_WIFI_AP1,_GLM_WIFI_PW1);
#endif

  int wifiConnect = 240;
  while ((wifiMulti.run() != WL_CONNECTED) && (wifiConnect-- > 0)) { // spend 2 minutes trying to connect to wifi
    // connecting to wifi
    delay(1000);
  }

  if (wifiMulti.run() != WL_CONNECTED ) { // still not connected? reboot!
    ESP.reset();
    delay(5000);
  }

  if (hasHostname) { // valid config found on FS, set network name
    WiFi.hostname(String(nodename)); // set network hostname
    ArduinoOTA.setHostname(nodename);  // OTA hostname defaults to esp8266-[ChipID]
    MDNS.begin(nodename); // set mDNS hostname
  }

  WiFi.macAddress(mac); // get esp mac address, store it in memory, build fw update url
  sprintf(macStr,"%x%x%x%x%x%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf(theURL,"/iotfw?mac=%s", macStr);

  // request latest config from web api
  if (!safeMode) getConfig();

  // check web api for new firmware
  if (!safeMode) httpUpdater();

  // start UDP for ntp client
  udp.begin(localPort);

  updateNTP();

  setSyncProvider(getNtptime); // use NTP to get current time
  setSyncInterval(600); // refresh clock every 10 min

  // start websockets server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // setup other things
  setupOTA();
  setupMQTT();

  // setup i2c if configured, basic sanity checking on configuration
  if (hasI2C && iotSDA>=0 && iotSCL>=0 && iotSDA!=iotSCL) {
    sprintf(str,"I2C enabled, using SDA=%u SCL=%u", iotSDA, iotSCL);
    mqtt.publish(mqttpub, str);

    Wire.begin(iotSDA, iotSCL); // from api config file

    //Wire.begin(12, 14); // from api config file
    i2c_scan();

    printConfig();

    if (hasRGB) setupRGB();
    if (hasIout) setupADS();
    if (hasSpeed) setupSpeed();

  }

  // OWDAT = 4;
  if (OWDAT>0) { // setup onewire if data line is using pin 1 or greater
    sprintf(str,"Onewire Data OWDAT=%u", OWDAT);
    mqtt.publish(mqttpub, str);
    oneWire.begin(OWDAT);
    if (hasTout) {
      ds18b20 = DallasTemperature(&oneWire);
      ds18b20.begin(); // start one wire temp probe
    }
    if (hasTpwr>0) {
      pinMode(hasTpwr, OUTPUT); // onewire power pin as output
      digitalWrite(hasTpwr, LOW); // ow off
    }
  }


  if (useMQTT) {
    String rebootReason = String("Last reboot cause was ") + rebootMsg;
    rebootReason.toCharArray(str, rebootReason.length()+1);
    mqtt.publish(mqttpub, str);
  }
}


void doVout() {
  int vBat=vccOffset;
  float voltage=0.00;
  String vStr;
  memset(voltsChr,0,sizeof(voltsChr));
  memset(adcChr,0,sizeof(adcChr));

  if (useGetvcc) {
    vBat += ESP.getVcc(); // internal voltage reference (Vcc);
    voltage = vBat / 1000.0;
    vStr = String("vcc=") + String(voltage,3);
  } else {
    vBat += analogRead(A0); // read the TOUT pin
    voltage = vBat * (vccDivisor / 1023.0); // adjust value, set 5.545 equal to your maximum expected input voltage
    vStr = String("bat=") + String(voltage,3);
  }
  sprintf(adcChr, "adc=%d", vBat);
  vStr.toCharArray(voltsChr, vStr.length()+1);
}

void doRGBout() {
  if(hasRGB) {
  	memset(str,0,sizeof(str));
  	sprintf(str, "red=%u,green=%u,blue=%u,white=%u", red, green, blue, white);
  	mqtt.publish(mqttpub, str);
  	wsSend(str);
  } else {
	mqtt.publish("RGB support not enabled.", str);
  }
  getRGB=false;
}

void doRSSI() {
  int rssi = WiFi.RSSI();
  memset(rssiChr,0,sizeof(rssiChr));
  sprintf(rssiChr, "rssi=%d", rssi);
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


  vStr = String("temp=") + String(temp,3);
  vStr.toCharArray(tmpChr, vStr.length()+1);
}

void doSpeed() {
  speedControl(fanSpeed, fanDirection);
}

void doIout() { // enable current reporting if module is so equipped
  if (hasRGB) return; // feature disabled if we're an rgb controller
  int16_t adc0, adc1, adc2, adc3;
  if (!hasI2C) return;

  memset(amps0Chr,0,sizeof(amps0Chr));
  memset(amps1Chr,0,sizeof(amps1Chr));
  memset(voltsChr,0,sizeof(voltsChr));

  adc0 = ads.readADC_SingleEnded(1) ; // adc channel 1 = switch 0 (switch one)
  raw0 = adc0;
  float voltage0 = (adc0 / 32767.0) * 4096.0;
  float amps0 = (voltage0 - ACSoffset) / mvPerA; // 44.0 = mv per amp from datasheet

  adc1 = ads.readADC_SingleEnded(0) ; // adc channel 0 = switch 1 (switch two)
  raw1 = adc1;
  float voltage1 = (adc1 / 32767.0) * 4096.0;
  float amps1 = (voltage1 - ACSoffset) / mvPerA; // 44.0 = mv per amp from datasheet

  adc2 = ads.readADC_SingleEnded(2) ; // adc channel 2 = battery voltage divider
  raw2 = adc2;
  float voltage2 = 0.0;
  if (altAdcvbat) voltage2 = (adc2 + vccOffset) / 1000.0;  // funky method for original current switch
  else voltage2 = (adc2 / 32767.0) * vccDivisor;  // proper method, 6.8k / 2.2k voltage divider

  if (amps0<0.080) amps0=0.0;
  if (amps1<0.080) amps1=0.0;
  if (voltage2<0.1) voltage2=0.0;
  char a0[9] = {};
  char a1[9] = {};
  char v2[9] = {};

  /*
  dtostrf(amps1, 6, 3, a1); // convert precision decimal 6 digits to string?
  a1[8] = '\0';
  sprintf(amps1Chr, "amps1=%s", a1); // copy strings together

  dtostrf(amps0, 6, 3, a0); // convert float to precision decimal 6 digits?
  a0[8] = '\0';
  sprintf(amps0Chr, "amps0=%s", a0); // copy strings together

  dtostrf(voltage2, 6, 3, v2); // convert float to precision decimal 6 digits?
  v2[8] = '\0';
  sprintf(voltsChr, "bat=%s", v2); // copy strings together
  */

  String tmp0 = String("amps0=") + String(amps0,3);
  tmp0.toCharArray(amps0Chr, tmp0.length()+1);

  String tmp1 = String("amps1=") + String(amps1,3);
  tmp1.toCharArray(amps1Chr, tmp1.length()+1);

  String tmp2 = String("bat=") + String(voltage2,3);
  tmp2.toCharArray(voltsChr, tmp2.length()+1);
}

void runUpdate() { // test for http update flag, received url via mqtt
  doUpdate = false; // clear flag
  updateCnt = 0; // clear update counter
  if (useMQTT && !hasRGB) {
    mqtt.publish(mqttpub, "Checking for updates");
    mqtt.loop();
  }
  delay(50);
  getConfig();
  httpUpdater();
}

void loop() {
  if (safeMode) { // safeMode engaged, enter blocking loop wait for an OTA update
    int safeDelay=30000; // five minutes in 100ms counts
    while (safeDelay--) {
      ArduinoOTA.handle();
      delay(100);
    }
    ESP.reset(); // restart, try again
    delay(5000); // give esp time to reboot
  }

  if(wifiMulti.run() != WL_CONNECTED) { // reboot if wifi connection drops
      ESP.reset();
      delay(5000);
  }

  if (!mqtt.connected()) {
    mqttreconnect(); // check mqqt status
  }

  if (hasRSSI) doRSSI();
  if (hasTout) doTout();
  if (hasVout) doVout();
  if (hasIout) doIout();
  if (getRGB) doRGBout();
  if (hasSpeed) doSpeed();

  if ( (doUpdate) || (updateCnt>= 60 / ((updateRate * 20) / 1000) ) ) runUpdate(); // check for config update as requested or every 60 loops

  if (wsConcount>0) wsData();
  if (useMQTT) mqttData(); // regular update for non RGB controllers
  if (prtConfig) printConfig(); // config print was requested

  sprintf(str,"Sleeping in %u seconds.", (updateRate*20/1000));
  if ((!skipSleep) && (sleepEn)) {
    if (useMQTT) mqtt.publish(mqttpub, str);
  }

  int cnt = 30;
  if (updateRate>30) cnt=updateRate;
  while(cnt--) {
    ArduinoOTA.handle();
    if (useMQTT) mqtt.loop();

    webSocket.loop();

    if (getTime) updateNTP(); // update time if requested by command
    if (scanI2C) i2c_scan();

    if (hasRGB) doRGB(); // rgb updates as fast as possible
    if (rgbTest) testRGB();

    if (setPolo) {
      setPolo = false; // respond to an mqtt 'ping' of sorts
      if (useMQTT) mqtt.publish(mqttpub, "Polo");
    }

    if (doReset) { // reboot on command
      if (useMQTT) {
        mqtt.publish(mqttpub, "Rebooting!");
        mqtt.loop();
      }
      delay(50);
      ESP.reset();
      delay(5000); // allow time for reboot
    }

    if (!hasRGB) delay(20); // don't delay for rgb controller
  }

  if ((!skipSleep) && (sleepEn)) {
    if ((sleepPeriod<60) || (sleepPeriod>43200)) sleepPeriod=900; // prevent sleeping for less than 1 minute or more than 12 hours
    sprintf(myChr,"Back in %d minutes", sleepPeriod/60);
    if (useMQTT) {
      mqtt.publish(mqttpub, myChr);
      mqtt.loop();
    }

    ESP.deepSleep(1000000 * sleepPeriod, WAKE_RF_DEFAULT); // sleep for 15 min
    delay(5000); // give esp time to fall asleep

  }
  skipSleep = false;
  updateCnt++;
}
