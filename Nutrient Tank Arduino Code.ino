// Nutrient Tank Arduino Code
// Written by Eoin Fitzgerald C13319946 - Last Update 24/06/2018
// Course/Year: DT021A/4
// Supervisor: Frank Duignan

// Include libraries

// Modbus
#include <SimpleModbusSlave.h>

// Temperature
#include <OneWire.h>
#include <DallasTemperature.h>


// Declarations

// Pump Relay
#define  relay 4  // pump relay pin
int relayState;   // varible for controlling pump from Modbus

// Temperature
#define ONE_WIRE_BUS 6                // temp data signal on pin 6
OneWire oneWire(ONE_WIRE_BUS);        // creates a oneWire object
DallasTemperature sensors(&oneWire);  // creates a DallasTemperatures (passes in oneWire object)
unsigned long SampleTime = 1000;      // sampleTime for taking readings once per second.

// pH
int ph_pin = A7;  // Pin 7 is Po
float Po;         // variable for reading pH signal
int pH;           // variable for processed pH signal

// Flow Rate
unsigned long count, start, _stop, flow;  // variables used for determiningflow rate of water

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
  TEMP,     // 9 Water Temperature
  RELAY,    // 10 Pump Relay
  FLOW,     // 11 Flow Rate
  PH,       // 12 pH

  TOTAL_REGS_SIZE
};
unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

void setup()
{
  // Start Serial Port
  Serial.begin(9600);

  // Modbus
  modbus_configure(9600, 2, 13, TOTAL_REGS_SIZE, 0);  // Modbus Configuration
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  // Temperature
  // Start up the library
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  sensors.requestTemperatures(); // Send the command to get initial temperature readings

  while (0) // loop to test length of requestTemperatures function
  {
    digitalWrite(11, LOW);
    sensors.requestTemperatures(); // Send the command to get temperature readings
    digitalWrite(11, HIGH);
    delay(10);
  }

  // Pump Relay
  pinMode(relay, OUTPUT); // sets the pin as output

  //Flow Rate
  pinMode(5, INPUT_PULLUP);
  TCCR1A = 0;           // Configuration of Timer/Counter Control Register A for normal operation
  TCCR1B = 0b11000111;  // Configuration of Timer/Counter Control Register B
  // noise cancellation, rising edge detect, normal operation (WGM12, WGM13),
  // external clock source (T1) rising edge  Serial.begin(9600);
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

  if (ledState)                     // set led
    digitalWrite(ledPin, HIGH);
  if (ledState == 0 ) // reset led
  {
    digitalWrite(ledPin, LOW);
    holdingRegs[LED_STATE] = 0;
  }

  // Pump Relay
  relayState = holdingRegs[RELAY];

  if (relayState == 2)
    digitalWrite(relay, HIGH);
  else if (ledState == 0 )
    digitalWrite(relay, LOW);


  // Flow Rate Estimation
  _stop = millis();           // takes the end time for calculating time between readings
  if (_stop - start > 1000)   // only enter once 1 second has elapsed since last entry,
    // allows for adequate counts to estimate counts per minute
  {
    count = TCNT1;            // current value in TCNT1 (16 bit register for Timer/Counter 1)
    TCNT1 = 0;                                // resets register for next count
    flow = (1000 * count) / (_stop - start);  // estimates pulses per second
    start = _stop;                            // sets start time to time entered to avoid calling millis()
    holdingRegs[FLOW] = flow;                 // Update Modbus Register
  }

  // pH
  int measure = analogRead(ph_pin);                 // read pH signal from sensor
  double voltage = 5.0 / 1024.0 * (double)measure;  // digital to voltage conversion

  Po = (7.0 + ((2.5 - voltage) / 0.18));        // voltage to pH conversion
  Po = (mapf(Po, 0.0, 6.1, 3.67, 10.35)) * 100; // calibration for more accurate pH readings
  // value is multiplied by 10 to pass to RPi while keeping accuracy
  pH = ((int)Po); // makes pH reading an integer to pass to RPi
  if (pH < 0)     // constrains pH to >= 0
  {
    pH = 0;
  }
  holdingRegs[PH] =  pH;  // Update Modbus Register

  // Temp
  if (millis() > SampleTime)     // only enters if 1 second has elapsed since last entry
  {
    holdingRegs[TEMP] = (sensors.getTempCByIndex(0)); // Update Holding Register
    sensors.requestTemperatures();                    // Request new temperature readings
    SampleTime = millis() + 1000;
  }

}

// Scaling function for scaling floating point numbers
double mapf(double val, double in_min, double in_max, double out_min, double out_max) // map function to deal with floats
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

