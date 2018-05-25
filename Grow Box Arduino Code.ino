// Grow Box Arduino Code
// Written by Eoin Fitzgerald C13319946 - Last Update 24/06/2018
// Course/Year: DT021A/4
// Supervisor: Frank Duignan

// Include libraries

// Modbus
#include <SimpleModbusSlave.h>

// Temperature
#include <OneWire.h>
#include <DallasTemperature.h>

// Light Relay
#include <BH1750FVI.h> // Sensor Library
#include <Wire.h>      // I2C Library

// Weight
#include <HX711.h>

// Pressure
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


// Declarations

// Temperature
#define ONE_WIRE_BUS 6                // temp data signal on pin 6
OneWire oneWire(ONE_WIRE_BUS);        // creates a oneWire object
DallasTemperature sensors(&oneWire);  // creates a DallasTemperatures (passes in oneWire object)
unsigned long SampleTime = 1000;      // sampleTime for taking readings once per second.

// Exhaust Fan
int pwmPin = 11;                        // digital pin 5
unsigned long count, start, _stop, rpm; // variables used for determining RPM of fan

// Intake Fan
#define fanRelay 3      // fan relay pin
int fanRelayState;      // varible for controlling fan from Modbus

// Weight
float weight = 0;                 // initial weight is set to 0
float calibration_factor = -7050; // value determined from calibration
#define DOUT  9                   // HX711 data (load cell)
#define CLK  8                    // Hx711 clk (load cell)
HX711 scale(DOUT, CLK);           // HX711 object

// Light Sensor
BH1750FVI LightSensor;  // light Sensor object

// Light Relay
#define  lightRelay 4   // light relay pin
int lightRelayState;    // varible for controlling lights from Modbus

// Pressure
float tempPressure, tempPressure2;  // temporary pressure variables
int pressure, pressure2;            // pressure reading variables
int pressureDiff;                   // pressure difference variable
Adafruit_BMP280 bmp;                // I2C pressure sensor object
Adafruit_BMP280 bmp2;               // I2C pressure sensor object
unsigned long SampleTime2 = 1000;   // sampleTime for taking readings once per second.

// P-Control
int simDiffPin = 2;   // analogue pin 2 used for pressure difference simulation
int simDiff = 0;      // store the value coming from the sensor
int error = 0;        // stores control error
int kp;               // P-Controller gain
int sp;               // set point
int pv;               // process variable
int co;               // Controller output
int mv;               // manipulated variable

#define  ledPin  2      // onboard led 
#define  buttonPin  7   // push button

// Modbus
enum
{
  ADC0,
  ADC1,
  ADC2,
  ADC3,
  ADC4,
  ADC5,
  LED_STATE,
  BUTTON_STATE,
  TOTAL_ERRORS,
  TEMP,     // 9 Air Temperature
  LUX,      // 10 Light Intesnity
  RPM,      // 11 RPM Reading
  PWM,      // 12 PWM Output
  KGS,      // Weight Reading
  LRELAY,   // Light Realy
  FRELAY,   // Fan Relay
  PDIFF,    // Pressure Difference
  PAS,      // Pressure 1
  PAS2,     // Pressure 2

  TOTAL_REGS_SIZE
};
unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

void setup()
{
  // Start Serial Port
  Serial.begin(9600);

  // Modbus
  modbus_configure(9600, 1, 13, TOTAL_REGS_SIZE, 0); // Modbus Configuration
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  // Temperature
  // Start up the library
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  sensors.requestTemperatures();      // Send the command to get initial temperature readings

  while (0) // loop to test length of requestTemperatures function
  {
    digitalWrite(11, LOW);
    sensors.requestTemperatures();
    digitalWrite(11, HIGH);
    delay(10);
  }

  // Light
  LightSensor.begin();                                // Starts Library
  LightSensor.SetAddress(Device_Address_H);           // Assigns I2C Address
  LightSensor.SetMode(Continuous_H_resolution_Mode);

  // Fan
  pinMode(pwmPin, OUTPUT);  // sets the pin as output
  kp = 1;                   // initialises proportional gain value
  sp = 0;                   // initialises set point value
  pv = 0;                   // initialises Process Variable value

  pinMode(5, INPUT_PULLUP);
  TCCR1A = 0;           // Configuration of Timer/Counter Control Register A for normal operation
  TCCR1B = 0b11000111;  // Configuration of Timer/Counter Control Register B
  // noise cancellation, rising edge detect, normal operation (WGM12, WGM13),
  // external clock source (T1) rising edge  Serial.begin(9600);

  // Weight
  scale.set_scale();
  scale.tare();                             //Reset the scale to 0
  long zero_factor = scale.read_average();  //Get a baseline reading

  // Light Relay
  pinMode(lightRelay, OUTPUT);  // sets Light Relay pin as output

  // Fan Relay
  pinMode(fanRelay, OUTPUT);    // sets Fan Relay pin as output


  // Pressure
  bmp.begin(0x76);                // Assigns address to pressure sensor
  bmp2.begin(0x77);               // Assigns address to pressure sensor
  pressure = bmp.readPressure();  // Gets initial pressure reading from sensor
  pressure = bmp2.readPressure(); // Gets initial pressure reading from sensor
}

void loop()
{
  // Modbus
  // Modbus updates by reading incoming data and performing assigned task
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
  for (byte i = 0; i < 6; i++)
  {
    holdingRegs[i] = analogRead(i);   // Holding Registers are updated
    delayMicroseconds(10);
  }
  byte buttonState = digitalRead(buttonPin); // read button states
  buttonState = 0;                           // force low as we have no button
  // assign the buttonState value to the holding register
  holdingRegs[BUTTON_STATE] = buttonState;

  // read the LED_STATE register value and set the onboard LED high or low with function 16
  byte ledState = holdingRegs[LED_STATE];

  if (ledState)       // set led
    digitalWrite(ledPin, HIGH);
  if (ledState == 0 ) // reset led
  {
    digitalWrite(ledPin, LOW);
    holdingRegs[LED_STATE] = 0;
  }

  // Light
  uint16_t lux = LightSensor.GetLightIntensity();   // Get Lux value
  holdingRegs[LUX] = lux;                           // Update Holding Register

  // Fan
  simDiff = analogRead(simDiffPin);       // read the value from the sensor
  pv = mapf(simDiff, 0, 1023, -8.0, 8.0); // scale input voltage to simulate +/- 8 hPa
  error = sp - pv;                        // calculate error
  co = kp * error;                        // calculate controller output
  mv = mapf(co, -8.0, 8.0, 0, 255);       // scale controller output for PWM (0-255)

  mv = constrain(mv, 0, 255); // limits mv to between 0 and 255 to constrain PWM output
  mv = (int)mv;               // makes mv and integer as holding registers passed to RPi need to be integer
  analogWrite(pwmPin, mv);    // pass PWM value to fan
  holdingRegs[PWM] = mv;      // Update Holding Register

  _stop = millis();         // takes the end time for calculating time between readings
  if (_stop - start > 1000) // only enter once 1 second has elapsed,
    // allows adequate counts to estimate RPM
  {
    count = TCNT1;          // current value in TCNT1 (16 bit register for Timer/Counter 1)
    rpm = (30000L * count) / (_stop - start); // estimates RPM
    TCNT1 = 0;                                // reset register value to 0 to restart timer
    start = _stop;                            // set start time as current time to restart timer
    holdingRegs[RPM] = rpm;                   // Update Modbus Register
  }

  // Load Sensor
  scale.set_scale(calibration_factor);  // Adjust to this calibration factor
  weight = scale.get_units();           // Reads weight from sensor
  weight = weight * 0.453592;           // Converts to grams
  if (weight < 0)                       // constrains weight to be >= 0
  {
    weight = 0;
  }
  holdingRegs[KGS] = weight * 1000.0;     // Update Holding Register

  // Light Relay
  lightRelayState = holdingRegs[LRELAY];  // Update Holding Register

  // switches Light Relay based on GUI
  if (lightRelayState == 2)
    digitalWrite(lightRelay, HIGH);       // Close Relay Contact
  else if (ledState == 0 )
    digitalWrite(lightRelay, LOW);        // Open Relay Contact

  // Fan Relay
  fanRelayState = holdingRegs[FRELAY];    // Update Holding Register

  // switches Fan Relay based on GUI
  if (fanRelayState == 2)
    digitalWrite(fanRelay, HIGH);         // Close Relay Contact
  else if (ledState == 0 )
    digitalWrite(fanRelay, LOW);          // Open Relay Contact

  // Temp
  if (millis() > SampleTime)    // only enters if 1 second has elapsed since last entry
  {
    holdingRegs[TEMP] = (sensors.getTempCByIndex(0)); // Update Holding Register
    sensors.requestTemperatures();                    // Request new temperature readings
    SampleTime = millis() + 1000;
  }

  // Pressure
  if (millis() > SampleTime2)   // only enters if 1 second has elapsed since last entry
  {
    pressure = ((int)tempPressure);         // makes reading an integer to be passed to RPi
    pressure2 = ((int)tempPressure2);       // makes reading an integer to be passed to RPi
    pressureDiff = pressure2 - pressure;    // determines pressure difference
    holdingRegs[PDIFF] = pressureDiff;      // Update Holding Register
    holdingRegs[PAS] = pressure;            // Update Holding Register
    holdingRegs[PAS2] = pressure2;          // Update Holding Register

    tempPressure2 = (bmp2.readPressure()) / 100.0;  // gets pressure reading and converts to hPa
    tempPressure = (bmp.readPressure()) / 100.0;    // gets pressure reading and converts to hPa
    SampleTime2 = millis() + 1000;
  }
}

// Scaling function for scaling floating point numbers
double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
