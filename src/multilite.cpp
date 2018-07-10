#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>

// uncomment for ac switch module, leave comment for dc switch module
// #define _ACMULTI true
#define _TRAILER true
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

const char sw1 = 4;
const char sw2 = 5;

char rssiChr[10];
char myChr[32];
uint8_t wifiDown = 0;
unsigned char mac[6];
char macStr[12];
char url[100];
char str[64];
char nodename[32];
char mqttServer[32], mqttpub[64],mqttsub[64], mqttbase[64];
int updateRate = 30;
unsigned char updateCnt = 0;
unsigned char mqttFail = 0;
bool safeMode = false;
bool coldBoot = true;
bool setPolo = false;

WiFiClient espClient;
PubSubClient mqtt(espClient);


void mqttPrintStr(char* _topic, char* myStr) {
  char myTopic[64];
  sprintf(myTopic, "%s/%s", mqttbase, _topic);
  mqtt.publish(myTopic, myStr);
}

void mqttPrintInt(char* myTopic, int myNum) {
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

  if (strcmp(cmdTxt, "marco")==0) setPolo = true; // respond to ping command
  else {
    uint8_t i = atoi(cmdVal);
    if (strcmp(cmdTxt, "ch1en")==0) {
      if (i == 1) { // ON
        digitalWrite(sw1, _ON); // nothing fancy for manual mode,
      } else { // OFF
        digitalWrite(sw1, _OFF); // nothing fancy for manual mode,
      }
    }
    else if (strcmp(cmdTxt, "ch2en")==0) {
      if (i == 1) { // ON
        digitalWrite(sw2, _ON); // nothing fancy for manual mode,
      } else { // OFF
        digitalWrite(sw2, _OFF); // nothing fancy for manual mode,
      }
    }
  }
}

// receive mqtt messages
void mqttCallBack(char* topic, byte* payload, unsigned int len) {
  char tmp[200];
  strncpy(tmp, (char*)payload, len);
  tmp[len] = '\0';
  handleCmd(tmp);
}

// maintain connection to mqtt broker
void mqttreconnect() {
  char tmp[200];

  int retry = 0;
  if (mqttFail>=100) { // repeated mqtt failure could mean network trouble, reboot esp
    doReset();
  }

  // Loop until we're reconnected
  while (!mqtt.connected()) {
    // Attempt to connect
    if (mqtt.connect(nodename)) {
      mqttFail=0; // reset fail counter
      // Once connected, publish an announcement...
      mqttPrintStr(mqttpub, "Hello, world!");
      // ... and resubscribe
      mqtt.subscribe(mqttsub);
    } else {
      // Wait before retrying
      delay(100);
    }
    if (retry++ > 4) { // try five times, return to main loop
      mqttFail++;
      return;
    }
  }
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
  mqtt.setServer("192.168.10.30", 1883); // setup mqtt broker connection
  mqtt.setCallback(mqttCallBack); // install function to handle incoming mqtt messages
  mqttreconnect(); // check mqqt status
}

void setup() {
    // if the program crashed, skip things that might make it crash
  String rebootMsg = ESP.getResetReason();
  if (rebootMsg=="Exception") safeMode=true;
  else if (rebootMsg=="Hardware Watchdog") safeMode=true;
  else if (rebootMsg=="Unknown") safeMode=true;
  else if (rebootMsg=="Software Watchdog") safeMode=true;
  else if (rebootMsg=="Deep-Sleep Wake") coldBoot=false;

  WiFi.begin("DXtrailer", "2317239216");

  int wifiConnect = 240;
  while ((WiFi.status() != WL_CONNECTED) && (wifiConnect-- > 0)) { // spend 2 minutes trying to connect to wifi
    // connecting to wifi
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED ) { // still not connected? reboot!
    ESP.reset();
    delay(5000);
  }

  WiFi.hostname(String("testsw1")); // set network hostname
  ArduinoOTA.setHostname("testsw1");  // OTA hostname defaults to esp8266-[ChipID]

  // setup other things
  setupOTA();
  setupMQTT();

  rebootMsg.toCharArray(str, rebootMsg.length()+1);
  mqttPrintStr("reboot/reason", str);

} // end of setup()


void doRSSI() {
  int rssi = WiFi.RSSI();
  mqttPrintInt("rssi", rssi);
}

void doPolo() {
  setPolo = false; // respond to an mqtt 'ping' of sorts
  mqttPrintStr(mqttpub, "Polo");
}

void loop() {
  doRSSI();

  int cnt = 30;
  while(cnt--) {
    if (setPolo) doPolo();

    ArduinoOTA.handle(); // handle OTA updates
    mqtt.loop(); // keep mqtt alive if enabled

    delay(200);
  }
}
