//code for Arduino Due
#include <PID_v1.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 adcOne;
#include <Adafruit_SSD1306.h>
#define OLED_RESET 13  // D13 on DUE
Adafruit_SSD1306 display(OLED_RESET);
//Timer variables
unsigned long previousMillis = 0;
const long interval = 500;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double kp = 10000, ki = 8000, kd = 0;// If running Proportional on Measurement, increasing P slows response.
//Mappping Values
double loadCellRaw = 0;
double pressureTRaw = 0;
double pressureCRaw = 0;
double stringGaugeRaw = 0;
double loadCell = 0;
double pressureT = 0;
double pressureC = 0;
double stringGauge = 0;

bool JogMode = false;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,P_ON_M, REVERSE);
void setup() {
Serial.begin(115200);
Serial1.begin(115200);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);    


analogWriteResolution(12);
analogWrite(DAC0, 2048);
pinMode(2, OUTPUT);
digitalWrite(2, LOW);
  // Initializes ADC and warns if ADC does not connenct
  if (!adcOne.begin(0x48)) {
    Serial.println("Failed to initialize ADC One.");
    while (1);
    Serial.println("ADC 1 OK");
  }
adcOne.setGain(GAIN_ONE);
adcOne.setDataRate(RATE_ADS1115_128SPS);  
myPID.SetMode(AUTOMATIC);
//myPID.SetOutputLimits(1548, 2548);
myPID.SetOutputLimits(0, 4095);
myPID.SetSampleTime(50);
//Serial.print("Setup Complete");
//Serial.println();
Serial.println("Setpoint,Input");
Setpoint = 18;
}

void loop() {
  // put your main code here, to run repeatedly:
GetPressure();
GetCylinderExtension();
GetLoadCell();  
Input = stringGauge;
ToDisplay();


myPID.Compute();
 if (!JogMode){
 analogWrite(DAC0, Output);
 }
 // analogWrite(DAC0, Output);
  unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print(Setpoint);
    Serial.print( ",");
    Serial.println(Input);
   // Serial.print( ",");
  //  Serial.print(Output);
   // Serial.println(JogMode);
    
  }
}


void printHelp()
{
  Serial.print( "setpoint:");
  Serial.println( Setpoint );

  Serial.print( "p:");
  Serial.println( kp );

  Serial.print( "i:");
  Serial.println( ki );

  Serial.print( "d:");
  Serial.println( kd );

  Serial.println();
}




String       inputString = "";         // a string to hold incoming data
String       logString = "";

void parseInput()
{
  String substr;
  //check_Serial_input();

  char first = inputString.charAt(0);

  switch (first)
  {
    case 'p':
      substr = inputString.substring(1);
      kp = substr.toDouble();
      Serial.print("Kp:");
      Serial.println(kp);
      myPID.SetTunings(kp, ki, kd);
      break;

    case 'i':
      substr = inputString.substring(1);
      ki = substr.toDouble();
      Serial.print("Ki:");
      Serial.println(ki);
      myPID.SetTunings(kp, ki, kd);
      break;

    case 'd':
      substr = inputString.substring(1);
      //      dGain = substr.toInt();
      //      Serial.print("Kd:");
      //      Serial.println(dGain);
      kd = substr.toDouble();
      Serial.print("Kd:");
      Serial.println(kd);
      //      writetoEEPROM();
      myPID.SetTunings(kp, ki, kd);
      break;
    // Stops the hydraulic pump
    case 's':
      Serial.println("Shutting off pump");
      digitalWrite(2, LOW);
      break;

    // Enables the hydraulic pump
    case 'g':
      Serial.println("Hydraulic pump set to go");
      digitalWrite(2, HIGH);
      break;

    // Jogs the piston in the compressive direction
    case 'c':
      Serial.println("testing jog in compressive direction");
      myPID.SetMode(MANUAL);
      JogMode = true;
      analogWrite(DAC0, 1548);
      break;

    // Jogs the piston in the tensile direction
    case 't':
      Serial.println("Testing Jog tenisle direction");
      myPID.SetMode(MANUAL);
      JogMode = true;
      analogWrite(DAC0, 2548);
      break;

    //holds the pid at current force value
    case 'h':
      Serial.println("testing holf");
      Setpoint = Input;
      myPID.SetMode(AUTOMATIC);
      JogMode = false;
      break;

    // Zeros the Force value
    case 'z':
      Serial.println("Zero Testing");
      //zeroPoint = InputRaw;
      Setpoint = 0;
      break;
    case 'm':
      substr = inputString.substring(1);
      //upperLimit = substr.toDouble();
      //fromHigh = InputRaw;
      break;
    case '\n':
      printHelp();
      break;
    case 'f':
      substr = inputString.substring(1);
      Setpoint = substr.toDouble();
      
      Serial.print("Setpoint:");
      Serial.println(Setpoint);
      break;
    default:
      Serial.println(inputString);
      Serial.println("Error: Shutting off pump");
      digitalWrite(2, LOW);
  }
  inputString = "";
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware Serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent1()
{
  while (Serial1.available())
  {
    // get the new byte:
    char inChar = (char)Serial1.read();
    //    Serial.print((int)inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      parseInput();
    }
  }
}
void ToDisplay()
    {
    display.clearDisplay();
    
    display.setCursor(0,0);
    display.print("0:");
    display.println(loadCellRaw,0);
    display.print("1:");
    display.print(pressureT,0);
    display.println("psi");
    display.print("2:");
    display.print(pressureC,0);
    display.println("psi");
    display.print("3:");
    display.print(stringGauge,1);
    display.println("in");
    display.display();
    }  
void GetPressure()
{
pressureTRaw = adcOne.readADC_SingleEnded(1);// read channel 1
pressureCRaw = adcOne.readADC_SingleEnded(2);// read channel 2
//map pressure sensors 1-5 Volts to 0 to 5000 psi
if (pressureTRaw < 8000) pressureT = 0; //8000 represents 1 volt. This line eliminates negative numbers toggling around zero psi.
else pressureT= map(pressureTRaw, 8000,40000, 0, 5000);//sensor outputs 1-5 volts over 0 - 5000PSI
if (pressureCRaw < 8000) pressureC = 0; //8000 represents 1 volt. This line eliminates negative numbers toggling around zero psi.
else pressureC= map(pressureCRaw, 8000,40000, 0, 5000);//sensor outputs 1-5 volts over 0 - 5000PSI   
}
void GetCylinderExtension()
{
  stringGaugeRaw = adcOne.readADC_SingleEnded(3);
  //maps cylinder extension from 0 - 35.75"
  stringGauge= map(stringGaugeRaw, 800,24000, 0, 35750); //maps cylinder extension from 0 - 35750mils"
  stringGauge = stringGauge/1000.0;//converts to inches
}
void GetLoadCell()
{
  loadCellRaw = adcOne.readADC_SingleEnded(0);
  loadCell= map(loadCellRaw, 0,32767, 0, 50000);
}
