#include <config.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//MQTT Queues
#define MQTT_PUB_IP "iot/" CONFIG_DEVICE_NAME "/ip"
#define MQTT_PUB_STATUS "iot/" CONFIG_DEVICE_NAME "/status"
#define MQTT_PUB_RSSI "iot/" CONFIG_DEVICE_NAME "/rssi"



WiFiClient espClient;
PubSubClient mqttClient(espClient);

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

long lastReconnectAttempt = 0;
long lastStatus = 0;

wl_status_t printWiFiStatus();
void setupWifi(bool blocking);
boolean connectToMqtt(bool blocking);
void onWifiConnect(const WiFiEventStationModeGotIP& event);
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event);
void processMqtt(bool blocking);
/*void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
*/


void onMqttMsgReceive(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}


/*
  once called at startup
*/

void setup() {
  //messure time consumed to startup
  unsigned long startMillis = millis();

  //set up pins and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //turn off LED is inverse

  // Serial port for debugging
  Serial.begin(74880);
  Serial.println("\n*-*-*- Start up of Device " CONFIG_DEVICE_NAME);
  //Serial.setDebugOutput(true); //debug for WiFi


  //MQTT handlers
  mqttClient.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  mqttClient.setCallback(onMqttMsgReceive);

  //connect to WiFi, use blocking if the application needs network; use non-blocking if the application can run without network
  setupWifi(false);

  Serial.printf("\nafter connectToWifi: %lums\n",(millis() - startMillis));


  //messure time consumed to startup
  Serial.printf("setup done after %lums\n",(millis() - startMillis));
}




/*
  Main loop
*/
void loop() {
  //needs to be called in short cycles. use blocking if the application needs MQTT; use non-blocking if the application can run without MQTT
  processMqtt(true);


  //do something nice every 5s. don't sleep in loop, since mqtt needs to be called in short cycles
  long now = millis();
  if (now - lastStatus > 5000) {
    lastStatus = now;
    printWiFiStatus();
    Serial.printf("local IP: %s, SSID: %s, AP: %s, RSSI: %s\n", WiFi.localIP().toString().c_str(), WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), String(WiFi.RSSI()).c_str());
    Serial.println();

    //send only if MQTT is connected
    if(mqttClient.connected()) {
      mqttClient.publish(MQTT_PUB_RSSI, String(WiFi.RSSI()).c_str());
      mqttClient.publish(MQTT_PUB_STATUS, "online");
    } else {
      Serial.println("no connection to MQTT at the moment");
    }
  }
}


void setupWifi(bool blocking) {
  //set correct mode of Wifi(Client mode)
  WiFi.mode(WIFI_STA);
  //Autoreconnect ist used for reconnecting in case of connection is lost
  WiFi.setAutoReconnect(true);
  Serial.printf("WiFi setup for MAC: %s, autoReconnect: %d\n", WiFi.macAddress().c_str(), WiFi.getAutoReconnect());

  //WiFi state handler
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // configure static IP address, if CONFIG_IP is set in config
  #ifdef CONFIG_IP
  // Set your Static IP configuration
  if(!WiFi.config(CONFIG_IP, CONFIG_GATEWAY, CONFIG_SUBNET, CONFIG_DNS1)) {
    Serial.println("Cloud NOT set fixed IP, switch to DHCP");
  } else {
    Serial.print("Fixed IP set to: ");
    Serial.println(CONFIG_IP);
  }
  #endif  
  
  // connect to Wi-Fi now
  Serial.print("trying to connect to WiFi SSID: " CONFIG_SSID "...\n my MAC address: ");
  Serial.println(WiFi.macAddress());
  
  //connect now
  WiFi.begin(CONFIG_SSID, CONFIG_WPA2_PASSWORD);
  
  //wait for connection if blocking is requested
  if(blocking) {
    while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println(".");
    }
  }
}

boolean connectToMqtt(bool blocking) {
  Serial.println("trying to connect to MQTT server...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);

  // Attempt to connect
  do {
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT.");
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");


      Serial.print("now: ");
      Serial.println(millis());
      Serial.println(timeClient.getFormattedTime());
      //initialize NTP
      timeClient.begin();
        
      timeClient.setTimeOffset(3600); // GMT +1 = 3600

      //update time
      timeClient.update();
      Serial.println(timeClient.getFormattedTime());
      Serial.print("now2: ");
      Serial.println(millis());

    } else {
      Serial.print("!!! Disconnected from MQTT, state: ");
      Serial.print(mqttClient.state());
      if(blocking) {
        delay(2000);
        Serial.println(".");
      }
      
    }
  } while (WiFi.isConnected() && !mqttClient.connected() && blocking);
  
  return mqttClient.connected();
}

/*
 makes sure we have a connection and processes the Messages received. triggers the call of the callback function
*/
void processMqtt(bool blocking) {
  //if not connected, connect to MQTT
  if (!mqttClient.connected()) {
    long now = millis();
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
  }
}





//called on sucessful connect to network
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  // print ESP8266 Local IP Address
  Serial.print("Connected to WiFi. local IP: ");
  Serial.println(event.ip);
}

//called on discounect from network and on each non-sucessfull reconnect (every second with setAutoReconnect(true)
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.printf("!!! Disconnected from WiFi. reason: %d, ",(int)event.reason);
  printWiFiStatus();
}



/*
Helper functions
*/

wl_status_t printWiFiStatus() {
    wl_status_t status = WiFi.status();
    Serial.print("WiFi Status: ");
    switch(status) {
        case WL_NO_SHIELD:
            Serial.println("WL_NO_SHIELD");
            return status;
        case WL_IDLE_STATUS:
            Serial.println("WL_IDLE_STATUS");
            return status;
        case WL_NO_SSID_AVAIL:
            Serial.println("WL_NO_SSID_AVAIL");
            return status;
        case WL_SCAN_COMPLETED:
            Serial.println("WL_SCAN_COMPLETED");
            return status;
        case WL_CONNECTED:
            Serial.println("WL_CONNECTED");
            return status;
        case WL_CONNECT_FAILED:
            Serial.println("WL_CONNECT_FAILED");
            return status;
        case WL_CONNECTION_LOST:
            Serial.println("WL_CONNECTION_LOST");
            return status;
        case WL_WRONG_PASSWORD:
            Serial.println("WL_WRONG_PASSWORD");
            return status;
        case WL_DISCONNECTED:
            Serial.println("WL_DISCONNECTED");
            return status;
        default:
            Serial.println("unknown");
            return status;
    }
    
  return status;
}  

