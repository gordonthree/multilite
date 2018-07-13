#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>

/*
extern "C"{
	#include "pwm.h"
	#include "user_interface.h"
}

////// PWM

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz

#define PWM_PERIOD 5000

// PWM channels

#define PWM_CHANNELS 3

// PWM setup (choice all pins that you use PWM)

uint32 io_info[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
//	{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
//	{PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
//	{PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
//	{PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
//	{PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D5
	{PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D6
	{PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D7
	{PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 ,15}, // D8
											   // D0 - not have PWM :-(
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];

// Dimmer variables

int16_t duty = 0;
int16_t step = 1;
*/

#define _ON 1
#define _OFF 0

const char sw1 = 5;
const char sw2 = 4;
const char sw3 = 0;

char str[64];
char mqttbase[64];
char mqttsub[100];

const char* nodename="frontswitch";
const char* myPub="msg";
const char* mySub="cmd";

const int mqttPort=1883;

#ifdef _TRAILER
const char* mqttroot="trailer";
const char* mqttServer="192.168.10.30";
const char* wifiSSID="DXtrailer";
const char* wifiPSK="2317239216";
#else
const char* mqttroot="home";
const char* mqttServer="192.168.2.30";
const char* wifiSSID="Tell my WiFi I love her";
const char* wifiPSK="2317239216";
#endif

unsigned char mqttFail = 0;
bool coldBoot = true;
bool setPolo = false;

int rgbRed=0, rgbGreen=0, rgbBlue=0;

long lastReconnectAttempt = 0;
long loopTimer = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);

void mqttPrintStr(const char* _topic, const char* myStr) {
  char myTopic[255];
  sprintf(myTopic, "%s/%s\0", mqttbase, _topic);
  mqtt.publish(myTopic, myStr);
}

void mqttPrintInt(const char* myTopic, const int myNum) {
  char myStr[8];
  sprintf(myStr, "%d\0", myNum);
  mqttPrintStr(myTopic, myStr);
}

void doReset() { // reboot on command
  mqttPrintStr(myPub, "Rebooting!");
  delay(50);
  ESP.reset();
  delay(5000); // allow time for reboot
}

void doRgbPwm() {
  // pwm_set_duty(rgbGreen, 0);
	// pwm_set_duty(rgbBlue, 1);
	// pwm_set_duty(rgbRed, 2);

	// pwm_start(); // commit
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
  if      (strcmp(topic, "home/rgb/red")==0)   rgbRed   = atoi((char*)payload);
  else if (strcmp(topic, "home/rgb/green")==0) rgbGreen = atoi((char*)payload);
  else if (strcmp(topic, "home/rgb/blue")==0)  rgbBlue  = atoi((char*)payload);
  else if (len>0) handleCmd(tmp);
}

boolean mqttReconnect() {
  char topic[100];
  if (mqtt.connect(nodename)) {
      // subscribe
      mqtt.subscribe(mqttsub);
      mqtt.subscribe("home/rgb/red");
      mqtt.subscribe("home/rgb/blue");
      mqtt.subscribe("home/rgb/green");

      // publish an announcement...
      mqttPrintStr(myPub, "Hello, world!");
      // mqttPrintInt("msg", mqttFail);
  }
  return mqtt.connected();
}

void setupMQTT() {
  sprintf(mqttbase, "%s/%s\0", mqttroot, nodename);
  sprintf(mqttsub, "%s/%s\0", mqttbase, mySub);

  mqtt.setServer(mqttServer, mqttPort); // setup mqtt broker connection
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

void setup() {
  pinMode(sw1, OUTPUT);
  pinMode(sw2, OUTPUT); 
//  pinMode(sw3, OUTPUT); 
  digitalWrite(sw1, _OFF);
  digitalWrite(sw2, _OFF);
//  digitalWrite(sw3, _OFF);

/*
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
*/

    // if the program crashed, skip things that might make it crash
  String rebootMsg = ESP.getResetReason();

  WiFi.begin(wifiSSID, wifiPSK);

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
  // doRgbPwm();

}
