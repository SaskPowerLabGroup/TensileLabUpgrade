
/*Jan 4 2022
   Added analogWriteResolution(12) to setup
   Added analogWrite dac0,2048 to set valve to zero volts
   Changed variable name InputMapped to InputRaw to reflect it is in A2D counts
   Changed upper limit to 253000 LBS @ 0 Volts to represent LBS.  Was running in Kg.




/* Features still needed: 
 *  Some way to detect failure
 *  Way to run pid off of the phidget gague mounted on the back
 */

#include <PID_v1.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 13  // D13 on DUE
#define PIN_SETPOINT A0
#define PIN_INPUT A2


//Include when using the ADC ADS1115
Adafruit_ADS1115 adcOne;
Adafruit_SSD1306 display(OLED_RESET);

//pressure variables
double pressureTRaw = 0;
double pressureCRaw = 0;
double pressureT = 0;
double pressureC = 0;

//Percentage drop in 0.5s that defines a failure
double failurePercent = 1; 

//Timer variables
unsigned long previousMillis = 0;
const long interval = 500;
double forceDifferential;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpointj, Inputj, Outputj;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
double kp = 1, ki = .5, kd = 0;// If running Proportional on Measurement, increasing P slows response.
double kpj = 10000, kij = 8000, kdj = 0;

//Mappping Values
double InputRaw = 0;
double zeroPoint = 20422; //Around 2.55 volts from the load cell converter or the zero point
double fromHigh = 0;
//double upperLimit = 253000;//0 volts out of load cell converter would represent 253000 LBS Tension
double upperLimit = 259135;
bool JogMode = true;
bool manualJog = false;

//PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,P_ON_M, DIRECT);
PID jogPID(&Inputj, &Outputj, &Setpointj,kpj,kij,kdj,P_ON_M, REVERSE);

double GetCylinderExtension()
{
  double stringGaugeRaw, stringGauge;
  stringGaugeRaw = adcOne.readADC_SingleEnded(3);
  //maps cylinder extension from 0 - 35.75"
  stringGauge= map(stringGaugeRaw, 800,24000, 0, 35750); //maps cylinder extension from 0 - 35750mils"
  stringGauge = stringGauge/1000.0;//converts to inches
  return stringGauge;
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
void setup()
{
Serial.begin(9600);
Serial.println("Setup Initalized");
Serial1.begin(9600);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
    
Serial.println("Display initialized");

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
  Setpoint = 1000;
  Setpointj = GetCylinderExtension();
  
  //turn the PID on

  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(1548, 2548);
  myPID.SetSampleTime(25);

  jogPID.SetMode(AUTOMATIC);
  jogPID.SetOutputLimits(0, 4095);
  jogPID.SetSampleTime(25);

  
  Serial1.println("Setpoint,Input");
  Serial.println("Setpoint,Input");
  Serial.println(Setpointj);
  Serial.println("Setup complete");
}

void loop()
{
  GetPressure();
  Inputj = GetCylinderExtension();
  InputRaw = adcOne.readADC_SingleEnded(0);
  Input = map(InputRaw, zeroPoint, fromHigh, 0, upperLimit);
  ToDisplay();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    forceDifferential = Input;
    Serial1.print(Setpointj);
    Serial1.print( ",");
    Serial1.print(Inputj);
    Serial1.print(",");
    Serial1.print(Setpoint);
    Serial1.print( ",");
    Serial1.print(Input);
    Serial1.print(",");
    Serial1.print(pressureT);
    Serial1.print( ",");
    Serial1.println(pressureC);

    Serial.print(Setpointj);
    Serial.print( ",");
    Serial.print(Inputj);
    Serial.print(",");
    Serial.print(Setpoint);
    Serial.print( ",");
    Serial.print(Input);
    Serial.print(",");
    Serial.print(pressureT);
    Serial.print( ",");
    Serial.println(pressureC);
    
}
  if(JogMode){
    jogPID.Compute();
    if(!manualJog){
      analogWrite(DAC0,Outputj);
    }
  }
  else{
    myPID.Compute();
    if(!manualJog){
      analogWrite(DAC0,Output);
    }
  }}


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
  //check_Serial1_input();

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
      //      Serial1.print("Kd:");
      //      Serial1.println(dGain);
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

    // Enables Force Mode
    case 'c':
      Serial.println("Enabling Force Pid");
      jogPID.SetMode(MANUAL);
      JogMode = false;
      myPID.SetMode(AUTOMATIC);
      InputRaw = adcOne.readADC_SingleEnded(0);
      manualJog = false;
      Setpoint = map(InputRaw, zeroPoint, fromHigh, 0, upperLimit);
      break;

    // Enables Jog Mode
    case 'j':
      Serial.println("JogMode enabled");
      myPID.SetMode(MANUAL);
      JogMode = true;
      jogPID.SetMode(AUTOMATIC);
      manualJog = false;
      Setpointj = GetCylinderExtension();
      break;

    //holds the pid at current force value
    case 'h':
      Serial.println("JogMode enabled, holding position");
      myPID.SetMode(MANUAL);
      JogMode = true;
      jogPID.SetMode(AUTOMATIC);
      manualJog = false;
      Setpointj = GetCylinderExtension();
      break;

    //extends piston cylinder
    case 'r':
      Serial.println("Retracting Piston");
      manualJog = true;
      myPID.SetMode(MANUAL);
      jogPID.SetMode(MANUAL);
      analogWrite(DAC0,2548);
      break;
      
    // retracts piston cylinder
    case 'e':
      manualJog = true;
      myPID.SetMode(MANUAL);
      jogPID.SetMode(MANUAL);
      analogWrite(DAC0,1548);
      break;
      
        
    // Zeros the Force value
    case 'z':
      Serial.println("Zero");
      zeroPoint = InputRaw;
      Setpoint = 0;
      break;
      
    case 'm':
      substr = inputString.substring(1);
      upperLimit = substr.toDouble();
      fromHigh = InputRaw;
      break;
    case '\n':
      printHelp();
      break;
      
    case 'f':
      substr = inputString.substring(1);
      if (JogMode){
        Setpointj = substr.toDouble();
      }
      else{
        Setpoint = substr.toDouble();
      }
      break;
    default:
      Serial.println(inputString);
      Serial.println("Error: Shutting off pump");
      digitalWrite(2, LOW);
  }
  inputString = "";
}

/*
  SerialEvent1 occurs whenever a new data comes in the
  hardware Serial1 RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent1()
{
  while (Serial1.available())
  {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      parseInput();
    }
  }
  Serial.println(inputString);
}
double smoothed(int numReadings)
{
  // forces value to be returned as an integer
  int averaged;

  // Averages the anoalog input numReadings amount of times
  double total = 0;
  for (int i = 0; i < numReadings ; i++) {
    total = total + adcOne.readADC_SingleEnded(0);
  }
  averaged = total / numReadings;
  return averaged;
}


void ToDisplay(){
    display.clearDisplay();

    display.setCursor(0,0);
    display.print("SPF:");
    display.println(Setpoint,0);
    display.print("PVF:");
    display.println(Input,0);   
    display.print("SPJ");
    display.println(Setpointj);
    display.print("PVJ");
    display.println(Inputj);
    display.println("");
    if(JogMode){
      display.print("Jog Mode");
    }
    else{
      display.print("Force Mode");
    }
    display.display();
}
