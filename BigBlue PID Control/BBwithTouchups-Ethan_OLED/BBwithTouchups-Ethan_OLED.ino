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

//Percentage drop from peak that defines a failure
double failurePercent = 0.70; 
double prevForce; 
//Timer variables. Set interval to how often the force is sent to the database
unsigned long previousMillis = 0;
const long interval = 500;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpointj, Inputj, Outputj;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
double kp = 1.5, ki = 2, kd = 0;// If running Proportional on Measurement, increasing P slows response.
double kpj = 3500, kij = 8000, kdj = 0;

//Mappping Values
double InputRaw = 0;
double zeroPoint = 20422;
//EEPROM.get(0, zeroPoint);
 //Around 2.55 volts from the load cell converter or the zero point
 //double upperLimit = 253000;//0 volts out of load cell converter would represent 253000 LBS Tension
double fromHigh = 0;
double upperLimit = 259135;


bool jogMode = true;
bool manualJog = false;
bool failureDetectionOn = true;
double jogLimit = 750;
double peakLoad = 0;

//PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,P_ON_M, DIRECT);
PID jogPID(&Inputj, &Outputj, &Setpointj,kpj,kij,kdj,P_ON_M, REVERSE);

void ToDisplay(){
  // updates the mini display on the due
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
    if(jogMode){
      display.print("Jog Mode");
    }
    else{
      display.print("Force Mode");
    }
    display.display();
}

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
//initializes hardware and data speeds
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
  adcOne.setDataRate(RATE_ADS1115_16SPS);
  Setpoint = 1000;
  Setpointj = GetCylinderExtension();
  
  //turn the PIDs on

  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(1548, 2548);
  myPID.SetSampleTime(50);

  jogPID.SetMode(AUTOMATIC);
  jogPID.SetOutputLimits(0, 4095);
  jogPID.SetSampleTime(50);

 
  Serial.println("Setpoint,Input");
  Serial.println(Setpointj);
  Serial.println("Setup complete");
}

void loop()
{
  //updates pressure, piston location and load variables
  GetPressure();
  Inputj = GetCylinderExtension();
  InputRaw = adcOne.readADC_SingleEnded(0);
  Input = map(InputRaw, zeroPoint, fromHigh, 0, upperLimit);
  ToDisplay();
  
  //posts to serial and serial1 main information
  //make posting a function and this will look alot cleaner
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
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
    Serial1.print(pressureC);
    Serial1.print( ",");
    Serial1.println(jogMode);

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
    Serial.print(pressureC);
    Serial.print( ",");
    Serial.println(jogMode);
    Serial.println(peakLoad);
}
  //updates peak load for failure detection
  if(Input>peakLoad){
    peakLoad = Input;
  }
  if(Setpoint < 100){
    Setpoint = 100;
  }
  if(Setpointj < 0){
    Setpointj = 0.25;
  }
  else if(Setpointj > 36){
    Setpointj = 35.75;
  }
  /*creates jog limit = the joglimit variable
  if((jogMode || manualJog)&&(abs(Input) > jogLimit)){
    manualJog = false;
    jogMode = false;
    Setpoint = Input;
    jogPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);
    Serial.print("Important Jog limit reached switching to force mode");
  }
  */
  
  //main jog mode PID
  if(jogMode){
    jogPID.Compute();
    if(!manualJog){
      analogWrite(DAC0,Outputj);
    }
  }
  //main force PID
  else{
    myPID.Compute();
    if(!manualJog){
      analogWrite(DAC0,Output);
    }
    //checks value against peak load and checks if failure has occured
    if((abs(Input)<(peakLoad*(1-failurePercent)))&&failureDetectionOn&&(prevForce > 750)){
      Setpointj = Inputj;
      jogMode = true;
      Serial.println(" lbs");
      peakLoad = 0;
      myPID.SetMode(MANUAL);
      jogPID.SetMode(AUTOMATIC);
      Serial.println("Failure Detected");
      Serial.println("Shutting off pump");
      digitalWrite(2, LOW);
    }
    prevForce = Input;
      
  }}


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
      myPID.SetMode(MANUAL);
      jogMode = true;
      jogPID.SetMode(AUTOMATIC);
      Setpointj = Inputj;
      digitalWrite(2, HIGH);
      break;

    // Enables Force Mode
    case 'c':
      Serial.println("Enabling Force Pid");
      jogPID.SetMode(MANUAL);
      jogMode = false;
      myPID.SetMode(AUTOMATIC);
      InputRaw = adcOne.readADC_SingleEnded(0);
      manualJog = false;
      Setpoint = map(InputRaw, zeroPoint, fromHigh, 0, upperLimit);
      failureDetectionOn = true;
      peakLoad = 0;
      break;

    // Enables Jog Mode
    case 'j':
      Serial.println("jogMode enabled");
      myPID.SetMode(MANUAL);
      jogMode = true;
      jogPID.SetMode(AUTOMATIC);
      manualJog = false;
      failureDetectionOn = false;
      Setpointj = GetCylinderExtension();
      peakLoad = 0;
      break;

    //holds the pid at current position
    case 'h':
      Serial.println("jogMode enabled, holding position");
      myPID.SetMode(MANUAL);
      jogMode = true;
      jogPID.SetMode(AUTOMATIC);
      manualJog = false;
      failureDetectionOn = false;
      Setpointj = GetCylinderExtension();
      peakLoad = 0;
      break;

    //retracts piston cylinder
    case 'r':
      Serial.println("Retracting Piston");
      manualJog = true;
      myPID.SetMode(MANUAL);
      jogPID.SetMode(MANUAL);
      analogWrite(DAC0,2548);
      failureDetectionOn = false;
      break;
      
    //extends piston cylinder
    case 'e':
      Serial.println("extending piston");
      manualJog = true;
      myPID.SetMode(MANUAL);
      jogPID.SetMode(MANUAL);
      analogWrite(DAC0,1548);
      failureDetectionOn = false;
      break;
      
        
    // Zeros the Force value
    case 'z':
      Serial.println("Zero");
      zeroPoint = InputRaw;
      //EEPROM.put(0, zeroPoint);
      Setpoint = 0;
      break;
      
    case 'm':
      Serial.println("max realigned");
      substr = inputString.substring(1);
      upperLimit = substr.toDouble();
      fromHigh = InputRaw;
      break;

    case 'l':
      peakLoad = 0;
      break;
       
    case 'f':
      substr = inputString.substring(1);
      if (jogMode){
        Setpointj = substr.toDouble();
      }
      else{
        Setpoint = substr.toDouble();
        if(Setpoint < Input){
          peakLoad = Setpoint;
          failureDetectionOn = false;
        }
        else{
          peakLoad = Input;
          failureDetectionOn = true;
        }}
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
{//writes serial data to a readable character array
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
