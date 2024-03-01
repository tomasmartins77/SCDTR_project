#include "pid.h"
#include "mcp2515.h"

const int LUMINAIRE = 1;

// luminaire constants
const int LED_PIN = 15;      // Pin that connects to the LED
const int ADC_PIN = A0;      // Pin that connects to the LDR
const int DAC_RANGE = 4095;  // Range of the DAC
const float vcc = 3.3;       // Maximum voltage
const int sampInterval = 10; // 100hZ OR 0.01s
const float m = -0.8;        // slope of the Lux-duty cycle curve
const float R = 10000;       // resistance of the LDR circuit
const float C = 10e-6;
const float b = log10(225000) + 0.8; // intercept of the Lux-duty cycle curve

// PID constants
pid my_pid{15, 10, 28, 10}; // K, b, Ti, Tt, Td, N
float r{1.0};               // reference

MCP2515 can0 {spi0, 22, 25, 21, 24, 10000000}; //channel spi0 or spi1

// luminaire variables
float occupancy_person = 7;    // occupancy reference luminance
float occupancy_no_person = 1; // no occupancy reference luminance
float dutyCycle_const;         // duty cycle for foward model
float lux;                     // luminance
int lux_counter;               // counter for the digital filter
float lux_sum;                 // sum for the digital filter
float E;                       // Energy
float V;                       // Luminance error
float fk;                      // Flicker
float powerMax = 0.254;        // Maximum power dissipated (0.108 of the LED and 0.146 of the resistor)
int counter;                   // counter for the performance metrics
float gain;                    // gain of the control system
float dutyCycle_k1;            // duty cycle for the performance metrics
float dutyCycle_k2;            // duty cycle for the performance metrics
bool streamL = false;          // stream luminance
bool streamD = false;          // stream duty cycle

unsigned long previousTime = 0; // will store last time LED was updated

struct DataPoint
{ // Structure to store the data
  float l;
  float d;
};
const int bufferSize = 5;                 // Size of the buffer
DataPoint last_minute_buffer[bufferSize]; // Buffer to store the data
int head = 0;                             // Index of the first empty position in the buffer

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);    // default is 10
  analogWriteFreq(60000);      // 60KHz, about max
  analogWriteRange(DAC_RANGE); // 100% duty cycle
  calibrate();                 // calibrate the gain

  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
}

void calibrate()
{
  delay(2000);
  analogWrite(LED_PIN, 0); // turn off the LED
  delay(2000);

  float lux1 = calculateLux(analogRead(ADC_PIN)); // read the lux

  analogWrite(LED_PIN, DAC_RANGE); // turn on the LED at 100% duty cycle
  delay(2000);

  float lux2 = calculateLux(analogRead(ADC_PIN)); // read the lux

  gain = (lux2 - lux1) / (1 - 0); // calculate the gain
}

void readSerial()
{
  static String buffer;

  while (Serial.available() > 0) // read the serial port
  {
    char c = Serial.read();
    if (c == '\n') // end of line
    {
      interface(buffer.c_str()); // process the command
      buffer = "";               // clear the buffer
    }
    else
    {
      buffer += c; // add the character to the buffer
    }
  }
}

void loop()
{
  unsigned long currentTime = millis();

  // Compute step and return if less than sampling period
  unsigned long h = (float)(currentTime - previousTime);
  if (h >= sampInterval)
  {
    // Control system
    controllerToLED(h);

    // String dataString = "r: " + String(r) + ", lux: " + String(lux);
    // Serial.println(dataString);

    Serial.print(lux);
    Serial.print(" ");
    Serial.print(my_pid.getDutyCycle());
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println("0 19");

    // Performance metrics
    performanceMetrics(h);

    if (streamL)
    {
      Serial.printf("s l %d %f %d\n", LUMINAIRE, lux, millis());
    }
    if (streamD)
    {
      Serial.printf("s d %d %f %d\n", LUMINAIRE, my_pid.getDutyCycle(), millis());
    }

    counter++;
    currentTime = resetVariables(currentTime);
  }
  else
  {
    digitalFilter(); // store values bewteen samples for better accuracy
  }
}

unsigned long resetVariables(unsigned long currentTime)
{
  previousTime = currentTime;
  lux_counter = 0;
  lux_sum = 0;
  return currentTime;
}

void controllerToLED(unsigned long h)
{
  int u;

  // Read from serial
  readSerial();
  // Digital filter
  digitalFilter();
  // Calculate lux average
  lux = lux_sum / lux_counter;
  // Control system
  if (my_pid.getFeedback())
  {
    u = (int)my_pid.computeControl(r, lux, h);
    // write the value to the LED
    analogWrite(LED_PIN, u);
    my_pid.setDutyCycle((float)u / DAC_RANGE);
    addToBuffer(lux, my_pid.getDutyCycle());
  }
  else
  {
    analogWrite(LED_PIN, dutyCycle_const * DAC_RANGE);
  }
}

void performanceMetrics(unsigned long h)
{
  E += my_pid.getDutyCycle() * (h / 1000); // Energy (without considering the power factor)

  V += max(0, r - lux); // Luminance error (no-mean value)

  if (counter > 1) // We need at least two samples to calculate the flicker
  {
    if ((my_pid.getDutyCycle() - dutyCycle_k1) * (dutyCycle_k1 - dutyCycle_k2) < 0)
    {
      fk += abs(my_pid.getDutyCycle() - dutyCycle_k1) + abs(dutyCycle_k1 - dutyCycle_k2); // Flicker
    }
    dutyCycle_k2 = dutyCycle_k1;
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 1)
  {
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 0)
  {
    dutyCycle_k2 = my_pid.getDutyCycle();
  }
}

void digitalFilter()
{
  // read the value from the ADC
  int read_adc = analogRead(ADC_PIN);
  // transform the ADC into lux
  float tempLux = calculateLux(read_adc);
  lux_counter++;
  lux_sum += tempLux;
}

float calculateLDR(float read_adc)
{
  float voltage = float(read_adc) / (DAC_RANGE / vcc); // ADC to voltage
  return (vcc * R) / voltage - R;
}

float calculateLux(float read_adc)
{
  return pow(10, (log10(calculateLDR(read_adc)) - b) / m); // resistance to lux
}

void addToBuffer(float value1, float value2)
{
  last_minute_buffer[head].l = value1;
  last_minute_buffer[head].d = value2;
  head = (head + 1) % bufferSize;
}
