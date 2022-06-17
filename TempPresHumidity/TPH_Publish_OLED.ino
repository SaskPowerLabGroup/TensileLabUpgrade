#include <Adafruit_SSD1306.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);
//Intended for Wemos D1 mini ESP32
//extractions from SHT3XD periodic mode and BMP085 example and https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <WiFi.h>
#include <PubSubClient.h>
// Replace the next variables with your SSID/Password combination
const char* ssid = "testlab";
const char* password = "HVLab001";
const char* mqtt_server = "192.168.50.2";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
ClosedCube_SHT31D sht3xd;
Adafruit_BMP085 bmp;
float temperature = 0;
float humidity = 0;
float pressure = 0;
const int ledPin = LED_BUILTIN;
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 48)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
void setup()
{
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);  
pinMode(ledPin, OUTPUT);
Wire.begin();
Serial.begin(9600);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

sht3xd.begin(0x45); // I2C address: 0x44 or 0x45
if (sht3xd.periodicStart(SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ) != SHT3XD_NO_ERROR)
    Serial.println("[ERROR] Cannot start periodic mode on SHT30");
if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }
 setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);  
}
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

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
 if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now; 
    printResult("SHT30", sht3xd.periodicFetchData());
    ToDisplay();
  //delay(1000);  
}
}
void printResult(String text, SHT31D result) {
  if (result.error == SHT3XD_NO_ERROR) {
    //Serial.println(text);
    Serial.print("T=");
    //Serial.print(result.t); 
    //temperature = bmp.readTemperature();
    // tempString[8];
    //dtostrf(temperature, 1, 2, tempString);
    //Serial.print("Temperature String: ");
    //Serial.println(tempString);
    temperature = (result.t);
    Serial.print(temperature);
    char tempString1[8];
    dtostrf(temperature, 1, 2, tempString1);
    client.publish("esp32/temperature", tempString1);
    Serial.print("C, P=");
    //Serial.print(bmp.readPressure());
    pressure = bmp.readPressure();
    pressure = pressure/100;
    Serial.print(pressure);
    char tempString2[10];
    dtostrf(pressure, 1, 2, tempString2);
    client.publish("esp32/pressure", tempString2);
    Serial.print("hPa, RH=");
    
    humidity = (result.rh);
    Serial.print(humidity);
    char tempString3[8];
    dtostrf(humidity, 1, 2, tempString3);
    client.publish("esp32/humidity", tempString3);
    
   
    Serial.println("%");
  } else {
    Serial.print(text);
    Serial.print(": [ERROR] Code #");
    Serial.println(result.error);
  }
}

  void ToDisplay(){
    display.clearDisplay();
    
    display.setCursor(0,0);
    display.print("T:");
    display.print(temperature);
    display.println(" C");
    display.print("P:");
    display.print(pressure);
    display.println(" H");
    display.print("H:");
    display.print(humidity);
    display.println(" %");
    display.display();
    
   
  }
