//Code for Wemos D1 Mini ESP32
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 adcOne;
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);
//Intended for Wemos D1 mini ESP32
//extractions from SHT3XD periodic mode and BMP085 example and https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>




// Replace the next variables with your SSID/Password combination
const char* ssid = "testlab";
const char* password = "HVLab001";
const char* mqtt_server = "192.168.50.10";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
ClosedCube_SHT31D sht3xd;
Adafruit_BMP085 bmp;
double temperature = 19.0;
double humidity = 21.1;
double pressure = 950.1;
double a2dReadV = 0;
double a2dReadC = 0;
double voltage = 0;
double current = 0;
double kvolt = 0;
const int ledPin = LED_BUILTIN;
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
// Data wire is plugged TO GPIO 5
#define ONE_WIRE_BUS 5

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 

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
Serial.begin(9600);  
sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();  
if (!adcOne.begin(0x48)) {
    Serial1.println("Failed to initialize ADC One.");
    while (1);
    Serial1.println("ADC 1 OK");
}  
adcOne.setGain(GAIN_FOUR);
adcOne.setDataRate(RATE_ADS1115_8SPS);
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);  
pinMode(ledPin, OUTPUT);
Wire.begin();

 // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
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

double map_float(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
 if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
//if (now - lastMsg > 5000) {
  if (now - lastMsg > 1000) {  
    lastMsg = now; 
    sensors.requestTemperatures(); // Send the command to get temperatures
  
  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.println(i,DEC);
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print("Temp C: ");
      Serial.print(tempC);
      Serial.print(" Temp F: ");
      Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
      char tempStringD[8];
      dtostrf(tempC, 1, 2, tempStringD);
      char channel[30] = "esp32/dallas";
      char stri[8];
      itoa(i,stri,10);
      strcat(channel,stri);
      client.publish(channel,tempStringD);
      
    }
  }
  printResult("SHT30", sht3xd.periodicFetchData());
      ReadVoltage();
      ReadCurrent();
      ToDisplay();
  }
}
void printResult(String text, SHT31D result) {
  if (result.error == SHT3XD_NO_ERROR) {
    //Serial.println(text);
    Serial.print("T=");
    temperature = (result.t);
    Serial.print(temperature);
    char tempString1[8];
    dtostrf(temperature, 1, 1, tempString1);
    client.publish("esp32/temperature", tempString1);
    Serial.print("C, P=");
    //Serial.print(bmp.readPressure());
    pressure = bmp.readPressure();
    pressure = pressure/100;
    Serial.print(pressure);
    char tempString2[10];
    dtostrf(pressure, 1, 1, tempString2);
    client.publish("esp32/pressure", tempString2);
    Serial.print("hPa, RH=");
    
    humidity = (result.rh);
    Serial.print(humidity);
    char tempString3[8];
    dtostrf(humidity, 1, 1, tempString3);
    client.publish("esp32/humidity", tempString3);
    Serial.print("%, C=");
    Serial.print(a2dReadV);
    Serial.print("c, V=");
    Serial.print(voltage);
    Serial.print("v, kV=");
    Serial.print(kvolt);
    Serial.print("kV,Current=");
    Serial.print("A,READ2_3=");
    Serial.print(a2dReadC);
    Serial.println("c");
    
    
    
    char tempString4[10];
    dtostrf(kvolt, 1, 1, tempString4);
    client.publish("esp32/voltage", tempString4);
    String stringV = tempString4;

    char tempString5[10];
    dtostrf(current, 1, 2, tempString5);
    client.publish("esp32/current", tempString5);
    
    client.publish("rts",tempString4);
   
    
   }
    
   else {
    Serial.print(text);
    Serial.print(": [ERROR] Code #");
    Serial.println(result.error);
  }
}

  void ToDisplay(){
    display.clearDisplay();
    
    display.setCursor(0,0);
    display.print("T:");
    display.print(temperature,1);
    display.println(" C");
    display.print("P:");
    display.print(pressure,1);
    display.println(" H");
    display.print("H:");
    display.print(humidity,1);
    display.println(" %");
    display.print("V:");
    display.print(kvolt,1);
    display.println("kV");
    display.print("C:");
    display.print(current,2);
    display.println("A");
    display.display();
    
   
  }
  void ReadVoltage(){
  a2dReadV = adcOne.readADC_Differential_0_1();
  voltage = map_float(a2dReadV, 0, 31992, 0, 400000);
  kvolt = voltage/1000.0;
  }
  void ReadCurrent(){
  a2dReadC = adcOne.readADC_Differential_2_3();
  current = map_float(a2dReadC, 0, 31992, 0, 2.0);
  

  }
  // function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}
