#include <WiFi.h>
#include <PubSubClient.h>

//select WEMOS D1 MINI ESP32
// Update these with values suitable for your network.
const char* ssid = "testlab";
const char* password = "HVLab001";
const char* mqtt_server = "192.168.50.10";
#define mqtt_port 1883
#define MQTT_USER "username"
#define MQTT_PASSWORD "password"
#define MQTT_SERIAL_PUBLISH_CH "bigBertha"
#define MQTT_SERIAL_RECEIVER_CH "bigBertha/inputs"
#define RXD2 16
#define TXD2 17


WiFiClient wifiClient;

PubSubClient client(wifiClient);

void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte *payload, unsigned int length) { 
    Serial2.write(payload, length);
    //Serial2.println();
    Serial.write(payload, length);
    //Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.setTimeout(500);// Set time out for 
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

// This is all that is required to publish to mqtt
void publishSerialData(char *serialData){
  if (!client.connected()) {
    reconnect();
  }
  client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}
void loop() {
   client.loop();
   if (Serial2.available() > 0) {
     char bfr[501];
     memset(bfr,0, 501);
     Serial2.readBytesUntil( '\n',bfr,500);
     publishSerialData(bfr);
   }
}
