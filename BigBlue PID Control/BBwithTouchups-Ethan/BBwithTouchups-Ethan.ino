
/*Jan 4 2022
   Added analogWriteResolution(12) to setup
   Added analogWrite dac0,2048 to set valve to zero volts
   Changed variable name InputMapped to InputRaw to reflect it is in A2D counts
   Changed upper limit to 253000 LBS @ 0 Volts to represent LBS.  Was running in Kg.

*/

/* Features still needed: 
 *  Some way to detect failure
 *  Way to run pid off of the phidget gague mounted on the back
 */

#include <PID_v1.h>
#include <Adafruit_ADS1X15.h>

//Include when using the ADC ADS1115
Adafruit_ADS1115 adcOne;
//Adafruit_ADS1115 adcTwo;

//#define PIN_OUTPUT 8
//#define PIN_PWM 9
//#define PIN_DIR 8
#define PIN_SETPOINT A0
#define PIN_INPUT A2

//Percentage drop in 0.5s that defines a failure
double failurePercent = 0.4 

//Timer variables
unsigned long previousMillis = 0;
const long interval = 500;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
double kp = 1, ki = .5, kd = 0;// If running Proportional on Measurement, increasing P slows response.

//Mappping Values
double InputRaw = 0;
double Input2Raw = 0;
double Input2 = 0;
double zeroPoint = 20422; //Around 2.55 volts from the load cell converter or the zero point
double fromHigh = 0;
//double upperLimit = 253000;//0 volts out of load cell converter would represent 253000 LBS Tension
double upperLimit = 259135;
bool JogMode = false;

//PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,P_ON_M, DIRECT);

void setup()
{
   Serial1.begin(115200);
   
analogWriteResolution(12);
  analogWrite(DAC0, 2048);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  // Initializes ADC and warns if ADC does not connenct
  if (!adcOne.begin(0x48)) {
    Serial1.println("Failed to initialize ADC One.");
    while (1);
    Serial1.println("ADC 1 OK");
  }
/* if (!adcTwo.begin(0x49)) {
  Serial1.println("Failed to initialize ADC Two.");
  while (1);
  Serial1.println("ADC 2 OK");
 }
*/

  adcOne.setGain(GAIN_ONE);
  adcOne.setDataRate(RATE_ADS1115_128SPS);
  Setpoint = 1000;


  //turn the PID on

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(1548, 2548);
  myPID.SetSampleTime(10);
  //Serial1.print("Setup Complete");
  //Serial1.println();
  Serial1.println("Setpoint,Input");
}

void loop()
{
  InputRaw = adcOne.readADC_SingleEnded(0);
  Input = map(InputRaw, zeroPoint, fromHigh, 0, upperLimit);
  
  Input2= map(Input2Raw, zeroPoint, fromHigh, 0, upperLimit);
  
  //Pressure = ads.readADC_SingleEnded(1);
  //Pressure1 = ads.readADC_SingleEnded(2);
  myPID.Compute();
  if (!JogMode) {
    analogWrite(DAC0, Output);
  }


  //posts to mqtt every 500 ms also checks for failure
  unsigned long currentMillis = millis();
  double forceDifferential
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    forceDifferential = Input
    Serial1.print(Setpoint);
    Serial1.print( ",");
    Serial1.println(Input);
  }

 if ((Input/forceDifferential) < failurePercent){
  Serial1.println("Failure Detected Shutting Off Pump");
  digitalWrite(2, LOW);
 }
  
  
}


void printHelp()
{
  Serial1.print( "setpoint:");
  Serial1.println( Setpoint );

  Serial1.print( "p:");
  Serial1.println( kp );

  Serial1.print( "i:");
  Serial1.println( ki );

  Serial1.print( "d:");
  Serial1.println( kd );

  Serial1.println();
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
      Serial1.print("Kp:");
      Serial1.println(kp);
      myPID.SetTunings(kp, ki, kd);
      break;

    case 'i':
      substr = inputString.substring(1);
      ki = substr.toDouble();
      Serial1.print("Ki:");
      Serial1.println(ki);
      myPID.SetTunings(kp, ki, kd);
      break;

    case 'd':
      substr = inputString.substring(1);
      //      dGain = substr.toInt();
      //      Serial1.print("Kd:");
      //      Serial1.println(dGain);
      kd = substr.toDouble();
      Serial1.print("Kd:");
      Serial1.println(kd);
      //      writetoEEPROM();
      myPID.SetTunings(kp, ki, kd);
      break;
    // Stops the hydraulic pump
    case 's':
      Serial1.println("Shutting off pump");
      digitalWrite(2, LOW);
      break;

    // Enables the hydraulic pump
    case 'g':
      Serial1.println("Hydraulic pump set to go");
      digitalWrite(2, HIGH);
      break;

    // Jogs the piston in the compressive direction
    case 'c':
      Serial1.println("testing jog in compressive direction");
      myPID.SetMode(MANUAL);
      JogMode = true;
      analogWrite(DAC0, 1548);
      break;

    // Jogs the piston in the tensile direction
    case 't':
      Serial1.println("Testing Jog tenisle direction");
      myPID.SetMode(MANUAL);
      JogMode = true;
      analogWrite(DAC0, 2400);
      break;

    //holds the pid at current force value
    case 'h':
      Serial1.println("testing holf");
      Setpoint = Input;
      myPID.SetMode(AUTOMATIC);
      JogMode = false;
      break;

    // Zeros the Force value
    case 'z':
      Serial1.println("Zero Testing");
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
      Setpoint = substr.toDouble();
      
      Serial1.print("Setpoint:");
      Serial1.println(Setpoint);
      break;
    default:
      Serial1.println(inputString);
      Serial1.println("Error: Shutting off pump");
      digitalWrite(2, LOW);
  }
  inputString = "";
}

/*
  Serial1Event occurs whenever a new data comes in the
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
    //    Serial1.print((int)inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      parseInput();
    }
  }
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
