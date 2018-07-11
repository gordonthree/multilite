#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>


#define _ON 1
#define _OFF 0

const char sw1 = 12;
const char sw2 = 13;

char rssiChr[10];
char myChr[32];
uint8_t wifiDown = 0;
unsigned char mac[6];
char macStr[12];

char str[64];
const char* nodename="testsw2";
const char* mqttbase="home/testsw2";
const char* mqttpub="home/testsw2/msg";
const char* mqttsub="home/testsw/cmd";

int updateRate = 30;
unsigned char updateCnt = 0;
unsigned char mqttFail = 0;
bool safeMode = false;
bool coldBoot = true;
bool setPolo = false;

long lastReconnectAttempt = 0;
long loopTimer = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);


void mqttPrintStr(const char* _topic, const char* myStr) {
  char myTopic[255];
  sprintf(myTopic, "%s/%s", mqttbase, _topic);
  mqtt.publish(myTopic, myStr);
}

void mqttPrintInt(const char* myTopic, const int myNum) {
  char myStr[8];
  sprintf(myStr, "%d", myNum);
  mqttPrintStr(myTopic, myStr);
}

void doReset() { // reboot on command
  mqttPrintStr(mqttpub, "Rebooting!");
  delay(50);
  ESP.reset();
  delay(5000); // allow time for reboot
}

void handleCmd(char* cmdStr) { // handle commands from mqtt or websocket
  // using c string routines instead of Arduino String routines ... a lot faster
  char* cmdTxt = strtok(cmdStr, "=");
  char* cmdVal = strtok(NULL, "=");

  if      (strcmp(cmdTxt, "marco")==0) setPolo = true; // respond to ping command
  else if (strcmp(cmdTxt, "ch1en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        digitalWrite(sw1, _ON); // nothing fancy for manual mode,
      } else { // OFF
        digitalWrite(sw1, _OFF); // nothing fancy for manual mode,
      }
    }
  }
  else if (strcmp(cmdTxt, "ch2en")==0) {
    if (cmdVal!=NULL) {
      uint8_t i = atoi(cmdVal);
      if (i == 1) { // ON
        digitalWrite(sw2, _ON); // nothing fancy for manual mode,
      } else { // OFF
        digitalWrite(sw2, _OFF); // nothing fancy for manual mode,
      }
    }
  }
}

// receive mqtt messages
void mqttCallBack(char* topic, uint8_t* payload, unsigned int len) {
  char tmp[len];
  strncpy(tmp, (char*)payload, len);
  tmp[len] = '\0';
  if (len>0) handleCmd(tmp);
}

boolean mqttReconnect() {
  if (mqtt.connect(nodename)) {
      // subscribe
      mqtt.subscribe(mqttsub);

      // publish an announcement...
      mqttPrintStr("msg", "Hello, world!");
      mqttPrintInt("msg", mqttFail);
  }
  return mqtt.connected();
}

void setupOTA() { // init arduino ide ota library
  ArduinoOTA.onStart([]() {
    //
  });
  ArduinoOTA.onEnd([]() {
    //
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //
  });

  ArduinoOTA.begin(); // start listening for arduinoota updates
}

void setupMQTT() {
  mqtt.setServer("192.168.2.30", 1883); // setup mqtt broker connection
  mqtt.setCallback(mqttCallBack); // install function to handle incoming mqtt messages
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

void setup() {
  pinMode(sw1, OUTPUT);
  pinMode(sw2, OUTPUT); 
  digitalWrite(sw1, _OFF);
  digitalWrite(sw2, _OFF);

    // if the program crashed, skip things that might make it crash
  String rebootMsg = ESP.getResetReason();
  if (rebootMsg=="Exception") safeMode=true;
  else if (rebootMsg=="Hardware Watchdog") safeMode=true;
  else if (rebootMsg=="Unknown") safeMode=true;
  else if (rebootMsg=="Software Watchdog") safeMode=true;
  else if (rebootMsg=="Deep-Sleep Wake") coldBoot=false;

  WiFi.begin("Tell my WiFi I love her", "2317239216");

  int wifiConnect = 240;
  while ((WiFi.status() != WL_CONNECTED) && (wifiConnect-- > 0)) { // spend 2 minutes trying to connect to wifi
    // connecting to wifi
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED ) { // still not connected? reboot!
    ESP.reset();
    delay(5000);
  }

  WiFi.hostname(String(nodename)); // set network hostname
  ArduinoOTA.setHostname(nodename);  // OTA hostname defaults to esp8266-[ChipID]

  // setup other things
  setupOTA();
  setupMQTT();

  rebootMsg.toCharArray(str, rebootMsg.length()+1);
  mqttPrintStr("reboot", str);

} // end of setup()


void doRSSI() {
  mqttPrintInt("rssi", WiFi.RSSI());
}

void doPolo() {
  setPolo = false; // respond to an mqtt 'ping' of sorts
  mqttPrintStr("msg", "Polo");
}

void loop() {
  long now = millis();
  if (now - loopTimer > 2000) { // every 2 seconds
    loopTimer=now;
    mqttPrintInt("uptime",now/1000);
    doRSSI();
  }

  if (setPolo) doPolo();

  ArduinoOTA.handle(); // handle OTA updates
  checkMQTT();
  
}
