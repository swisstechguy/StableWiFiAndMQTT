#include "MqttTopicFunctionMap.h"
#include <config.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <PubSubClient.h>

//MQTT Queues
#define MQTT_PUB_CMD_LED "iot/" CONFIG_DEVICE_NAME "/command/led"
#define MQTT_PUB_CMD_MSG "iot/" CONFIG_DEVICE_NAME "/command/msg"

long lastJob = 0;

/* 
  Base Client with WiFi and MQTT 
 */

//MQTT status updates are sent by base client
#define MQTT_PUB_STATUS_IP "iot/" CONFIG_DEVICE_NAME "/status/ip"
#define MQTT_PUB_STATUS_UPTIME "iot/" CONFIG_DEVICE_NAME "/status/uptime"
#define MQTT_PUB_STATUS_MSG "iot/" CONFIG_DEVICE_NAME "/status/msg"
#define MQTT_PUB_STATUS_RSSI "iot/" CONFIG_DEVICE_NAME "/status/rssi"
#define MQTT_PUB_STATUS_HEAP "iot/" CONFIG_DEVICE_NAME "/status/heap"

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

struct MqttTopicFunctionMap mqttCommandMap;

void setupMqtt();
void setupWifi(bool blocking);
boolean connectToMqtt(bool blocking);
void loopMqtt(bool blocking);
void onMqttMsgReceive(char* topic, byte* payload, unsigned int length);
void onWifiConnect(const WiFiEventStationModeGotIP& event);
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event);
wl_status_t printWiFiStatus();
long lastReconnectAttempt = 0;
long lastMqttStatus = 0;

/*
Switch Debug output to Serial-Console ON and OFF in config.h
*/
#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#define D_printf(...)  Serial.printf(__VA_ARGS__)
#define D_write(...)    Serial.write(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_printf(...)
#define D_println(...)
#define D_write(...)
#endif

/*
 Custom Functions for MQTT Commands
*/
void displayMsg(const char* payload, bool payload_bool, int payload_int, float payload_float) {
    Serial.printf("Hello, %s\n", payload);
}

void led(const char* payload, bool payload_bool, int payload_int, float payload_float) {
  // Switch on the LED if an 1 was received as first character
  if (payload_bool) {
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}





/*
  setup: once called at startup
*/

void setup() {
  //messure time consumed to startup
  #if DEBUG
  unsigned long startMillis = millis();
  #endif

  //set up pins and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //turn off LED is inverse

  // Serial port for debugging
  Serial.begin(74880);
  D_println("\n*-*-*- Start up of Device " CONFIG_DEVICE_NAME);
  //Serial.setDebugOutput(true); //debug for WiFi

  //prepair MQTT client
  setupMqtt();

  //subscribe to the MQTT topics and give a reference to the functions. This function will be called as soon as we receive a mqtt msg for this topic. loopMqtt will process it.
  //initializeMap(&mqttCommandMap, 0);
  subscribeTopic(&mqttCommandMap, MQTT_PUB_CMD_MSG, displayMsg);
  subscribeTopic(&mqttCommandMap, MQTT_PUB_CMD_LED, led);


  //connect to WiFi, use blocking if the application needs network; use non-blocking if the application can run without network. with blocking we wait here for a connection
  setupWifi(false);

  //messure time consumed to startup
  D_printf("setup done after %lums\n",(millis() - startMillis));

}


/*
  Main loop
  
  IMPORTANT: don't sleep in loop, since mqtt needs to be called in short cycles

*/
void loop() {
  //needs to be called in short cycles. use blocking if the application needs MQTT; use non-blocking if the application can run without MQTT
  loopMqtt(false);


  //log status every 5s. don't sleep in loop, since mqtt needs to be called in short cycles
  long now = millis();
  if (now - lastJob > 60*1000) {
    lastJob = now;
    printWiFiStatus();
    Serial.printf("local IP: %s, SSID: %s, AP: %s, RSSI: %s\n", WiFi.localIP().toString().c_str(), WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), String(WiFi.RSSI()).c_str());
    Serial.println();
  


    //send only if MQTT is connected
    if(mqttClient.connected()) {

    } else {
      Serial.println("no connection to MQTT at the moment");
    }
  }
}



/*
 
 Methods for Base Client with WiFi and MQTT

*/

void setupMqtt() {
 //MQTT handlers
  mqttClient.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  mqttClient.setCallback(onMqttMsgReceive);
}

void setupWifi(bool blocking) {
  //messure time consumed to startup
  #if DEBUG
  unsigned long startMillis = millis();
  #endif
  //set correct mode of Wifi(Client mode)
  WiFi.mode(WIFI_STA);
  //Autoreconnect ist used for reconnecting in case of connection is lost
  WiFi.setAutoReconnect(true);
  D_printf("WiFi setup for MAC: %s, autoReconnect: %d\n", WiFi.macAddress().c_str(), WiFi.getAutoReconnect());

  //WiFi state handler
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // configure static IP address, if CONFIG_IP is set in config
  #ifdef CONFIG_IP
  // Set your Static IP configuration
  if(!WiFi.config(CONFIG_IP, CONFIG_GATEWAY, CONFIG_SUBNET, CONFIG_DNS1)) {
    D_println("Cloud NOT set fixed IP, switch to DHCP");
  } else {
    D_print("Fixed IP set to: ");
    D_println(CONFIG_IP);
  }
  #endif  
  
  // connect to Wi-Fi now
  D_print("trying to connect to WiFi SSID: " CONFIG_SSID "...\n my MAC address: ");
  D_println(WiFi.macAddress());
  
  //connect now
  WiFi.begin(CONFIG_SSID, CONFIG_WPA2_PASSWORD);
  
  //wait for connection if blocking is requested
  if(blocking) {
    while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      D_println(".");
    }
    D_printf("\nafter connectToWifi: %lums\n",(millis() - startMillis));
  }
}

boolean connectToMqtt(bool blocking) {
  D_println("trying to connect to MQTT server...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);

  // Attempt to connect
  do {
    if (mqttClient.connect(clientId.c_str())) {
      //reconnected, send status msg
      long now = millis();
      char status[200]; 
      sprintf(status, "reconnected. uptime %lds IP: %s, SSID: %s, AP: %s, RSSI: %s, FREE_HEAP: %d\n", now/1000, WiFi.localIP().toString().c_str(), WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), String(WiFi.RSSI()).c_str(), ESP.getFreeHeap()); 
      D_printf(status);
      mqttClient.publish(MQTT_PUB_STATUS_MSG, status);

      //subscribe to all registered topics
      D_printf("connected, subscribe to all topics\n");
      const char** keys = getAllTopics(&mqttCommandMap);
      for (size_t i = 0; i < mqttCommandMap.size; ++i) {
        D_printf("subscribe to %s\n", keys[i]);
        mqttClient.subscribe(keys[i]);
      }      

    } else {
      D_print("!!! Disconnected from MQTT, state: ");
      D_print(mqttClient.state());
      D_println();
      if(blocking) {
        delay(2000);
        D_println(".");
      }
      
    }
  } while (WiFi.isConnected() && !mqttClient.connected() && blocking);
  
  return mqttClient.connected();
}

/*
 makes sure we have a connection and processes the Messages received. triggers the call of the callback function
*/
void loopMqtt(bool blocking) {
  long now = millis();
  //if not connected, connect to MQTT
  if (!mqttClient.connected()) {
    
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (connectToMqtt(false)) {
        lastReconnectAttempt = 0;

      }
    }
  } else {
    // MQTT connected, process the messages
    mqttClient.loop();

    //give status on MQTT
    if (now - lastMqttStatus > CONFIG_MQTT_STATUS_REPORT*1000) {
      lastMqttStatus = now;
      mqttClient.publish(MQTT_PUB_STATUS_RSSI, String(WiFi.RSSI()).c_str());
      mqttClient.publish(MQTT_PUB_STATUS_UPTIME, String(now/1000).c_str());
      mqttClient.publish(MQTT_PUB_STATUS_HEAP, String(ESP.getFreeHeap()).c_str());
    
    }
  }
}


void onMqttMsgReceive(char* topic, byte* payload, unsigned int length) {
  D_print("Message arrived [");
  D_print(topic);
  D_println("] ");
  
  //Attention: payload is a byte array and not a \0-terminated string!

  //make a string from payload
  //copy payload into char array and add \0 to make it a string
  char str_payload[length+1];
  str_payload[length] = '\0';
  for (unsigned int i = 0; i < length; i++) {
    str_payload[i] = (char)payload[i];
  }
  //make a int from payload
  int int_payload = atoi(str_payload);
  //make a boolean from payload
  bool bool_payload = strcasecmp("on", str_payload) == 0;
  //make a float from payload
  float float_payload = atof(str_payload);

  //call the registered function
  const FunctionPointer func = getFunctionByTopic(&mqttCommandMap, topic);
  if (func != NULL) {
      D_printf("calling function(%s, %d, %d, %f)\n", str_payload, bool_payload, int_payload, float_payload);
      func(str_payload, bool_payload, int_payload, float_payload);
  } else {
      printf("function for topic '%s' not found\n", topic);
  }
}


//called on sucessful connect to network
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  // print ESP8266 Local IP Address
  D_print("Connected to WiFi. local IP: ");
  D_println(event.ip);
}

//called on discounect from network and on each non-sucessfull reconnect (every second with setAutoReconnect(true)
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  D_printf("!!! Disconnected from WiFi. reason: %d, ",(int)event.reason);
  printWiFiStatus();
}



/*
Helper functions
*/

wl_status_t printWiFiStatus() {
    wl_status_t status = WiFi.status();
    D_print("WiFi Status: ");
    switch(status) {
        case WL_NO_SHIELD:
            D_println("WL_NO_SHIELD");
            return status;
        case WL_IDLE_STATUS:
            D_println("WL_IDLE_STATUS");
            return status;
        case WL_NO_SSID_AVAIL:
            D_println("WL_NO_SSID_AVAIL");
            return status;
        case WL_SCAN_COMPLETED:
            D_println("WL_SCAN_COMPLETED");
            return status;
        case WL_CONNECTED:
            D_println("WL_CONNECTED");
            return status;
        case WL_CONNECT_FAILED:
            D_println("WL_CONNECT_FAILED");
            return status;
        case WL_CONNECTION_LOST:
            D_println("WL_CONNECTION_LOST");
            return status;
        case WL_WRONG_PASSWORD:
            D_println("WL_WRONG_PASSWORD");
            return status;
        case WL_DISCONNECTED:
            D_println("WL_DISCONNECTED");
            return status;
        default:
            D_println("unknown");
            return status;
    }
    
  return status;
}  

