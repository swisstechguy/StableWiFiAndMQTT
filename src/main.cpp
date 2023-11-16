#include <config.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <PubSubClient.h>

//MQTT Queues
#define MQTT_PUB_IP "iot/" CONFIG_DEVICE_NAME "/ip"
#define MQTT_PUB_STATUS "iot/" CONFIG_DEVICE_NAME "/status"
#define MQTT_PUB_RSSI "iot/" CONFIG_DEVICE_NAME "/rssi"



WiFiClient espClient;
PubSubClient mqttClient(espClient);

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

long lastReconnectAttempt = 0;

wl_status_t printWiFiStatus();
void setupWifi(bool blocking);
boolean connectToMqtt();
void onWifiConnect(const WiFiEventStationModeGotIP& event);
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event);
/*void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
*/


void callback(char* topic, byte* payload, unsigned int length) {
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


  //MQTT state handlers
  mqttClient.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  mqttClient.setCallback(callback);

/*
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  // If your broker requires authentication(username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
 
  //mqttClient.onMessage(onMqttMessage);
*/

  //connect to WiFi, use blocking if the application needs network; use non-blocking if the application can run without network
  setupWifi(true);

  Serial.printf("\nafter connectToWifi: %lums\n",(millis() - startMillis));


  //messure time consumed to startup
  Serial.printf("setup done after %lums\n",(millis() - startMillis));
}




/*
  Main loop
*/
void loop() {
  Serial.print("loop: ");

  //connect to MQTT
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (connectToMqtt()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    mqttClient.loop();
  }


  
  printWiFiStatus();
  
  if(WiFi.isConnected()) {
    Serial.printf("local IP: %s, SSID: %s, AP: %s, RSSI: %s\n", WiFi.localIP().toString().c_str(), WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), String(WiFi.RSSI()).c_str());

    if(mqttClient.connected()) {
      mqttClient.publish(MQTT_PUB_RSSI, String(WiFi.RSSI()).c_str());
      mqttClient.publish(MQTT_PUB_STATUS, "online");
    } else {
      Serial.println("no connection to MQTT at the moment");
    }
  } else {
    Serial.println("no network connection at the moment");
  }
  Serial.println();
  delay(5000);
}

/*
  Network and MQTT functions
*/

/*
  prepairs the network and connects to WiFi.
  call it only once in setup
*/
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

boolean connectToMqtt() {
  Serial.println("trying to connect to MQTT server...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);

  // Attempt to connect
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    mqttClient.publish("outTopic", "hello world");
    // ... and resubscribe
    mqttClient.subscribe("inTopic");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    
  }
  return mqttClient.connected();
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
void onMqttConnect(bool sessionPresent) {
  Serial.println("connected to MQTT server.");
  String strIp = String(WiFi.localIP()[0]) + "." + String(WiFi.localIP()[1]) + "." + String(WiFi.localIP()[2]) + "." + String(WiFi.localIP()[3]);
  
  //publish status and IP
  mqttClient.publish(MQTT_PUB_STATUS, 0, false, "online");
  mqttClient.publish(MQTT_PUB_IP, 0, true, String(strIp).c_str());
  
  //subscribe to topics
 // mqttClient.subscribe(MQTT_SUB_CMD_FAN, 0);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.printf("!!! Disconnected from MQTT. reason: %d, ",(int)reason);
  printWiFiStatus();

  //try to reconnect in 5s if the network is present. if it fails, this method will be called again.
  if(WiFi.isConnected()) {
    mqttReconnectTimer.once(5, connectToMqtt);
  }
}
*/
/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}*/




/*
Helper functions
*/

wl_status_t printWiFiStatus() {
    wl_status_t status = WiFi.status();
    Serial.print("WiFi Status");
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
