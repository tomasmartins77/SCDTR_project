#include "pid.h"
#include "mcp2515.h"
#include <hardware/flash.h>
#include <Arduino.h>

const int LUMINAIRE = 1;

// luminaire constants
const int LED_PIN = 15;       // Pin that connects to the LED
const int ADC_PIN = A0;       // Pin that connects to the LDR
const float DAC_RANGE = 4095; // Range of the DAC
const float vcc = 3.3;        // Maximum voltage
const int sampInterval = 10;  // 100hZ OR 0.01s
const float m = -0.9;         // slope of the Lux-duty cycle curve
const float R = 10000;        // resistance of the LDR circuit
const float C = 10e-6;
const float b = log10(225000) + 0.9; // intercept of the Lux-duty cycle curve

// PID constants
pid my_pid{1000, 0.1, 0.1, 0.1}; // K, b, Ti, Tt, Td, N
float r{3.0};                    // reference

// luminaire variables
float occupancy_person = 10;   // occupancy reference luminance
float occupancy_no_person = 3; // no occupancy reference luminance
float volt;                    // voltage
float E;                       // Energy
float V;                       // Luminance error
float fk;                      // Flicker
float powerMax = 0.1271;       // Maximum power dissipated (0.108 of the LED and 0.0191 of the resistor)
int counter;                   // counter for the performance metrics
float gain;                    // gain of the control system
float dutyCycle_k1;            // duty cycle for the performance metrics
float dutyCycle_k2;            // duty cycle for the performance metrics
bool streamL = false;          // stream luminance
bool streamD = false;          // stream duty cycle
int linear = 0;
int visualize = 0;
unsigned long time_now = 0;
bool dFunction = false;
float sumVolt = 0;
int countVolt = 0;

struct DataPoint
{ // Structure to store the data
  float l;
  float d;
};
const int bufferSize = 6000;              // Size of the buffer
DataPoint last_minute_buffer[bufferSize]; // Buffer to store the data
int head = 0;                             // Index of the first empty position in the buffer

unsigned long int timer_time{0};
volatile bool timer_fired{false};
struct repeating_timer timer;

MCP2515 can0{spi0, 17, 19, 16, 18, 10000000}; // channel spi0 or spi1
uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx{0}, counterRx{0};
MCP2515::ERROR err;
unsigned long time_to_write;
unsigned long write_delay{1000};
const byte interruptPin{20};
volatile byte data_available{false};

float test = 0;

bool my_repeating_timer_callback(struct repeating_timer *t)
{
  if (!timer_fired)
    timer_fired = true;
  return true;
}

void read_interrupt(uint gpio, uint32_t events)
{
  data_available = true;
}

void setup()
{
  flash_get_unique_id(this_pico_flash_id);
  node_address = this_pico_flash_id[6];
  Serial.begin(115200);
  analogReadResolution(12);    // default is 10
  analogWriteFreq(60000);      // 60KHz, about max
  analogWriteRange(DAC_RANGE); // 100% duty cycle
  calibrate();                 // calibrate the gain

  add_repeating_timer_ms(-10, my_repeating_timer_callback, NULL, &timer); // 100 Hz

  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);
  time_to_write = millis() + write_delay;
}

void calibrate()
{
  delay(2000);
  analogWrite(LED_PIN, 0); // turn off the LED
  delay(3000);

  float volt1 = calculateVoltage(analogRead(ADC_PIN)); // read the lux

  analogWrite(LED_PIN, DAC_RANGE); // turn on the LED at 100% duty cycle
  delay(3000);

  float volt2 = calculateVoltage(analogRead(ADC_PIN)); // read the lux

  gain = (volt2 - volt1) / (4095 - 0); // calculate the gain
  r = calculateLux2Voltage(r);      // set the reference
  my_pid.setB(0.9*(1 / (gain * my_pid.getK())));
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
  controlLoop();
  // receiveMessage();
  // sendMessage();
}

void controllerToLED()
{
  int pwm;
  float u;

  // read the value from the ADC
  int read_adc = analogRead(ADC_PIN);

  // transform the ADC into lux
  float voltTemp = calculateVoltage(read_adc);
  volt = (sumVolt + voltTemp) / (countVolt + 1);
  float LDR = calculateLDR(read_adc);

  my_pid.setTi((LDR * R) / (LDR + R) * C);

  // Control system
  u = my_pid.computeControl(r, volt);
  pwm = (int)u;
  // set the duty cycle
  my_pid.setDutyCycle(u / DAC_RANGE);
  // Add to the buffer
  addToBuffer(calculateVoltage2Lux(volt), my_pid.getDutyCycle());
  // write the value to the LED
  analogWrite(LED_PIN, pwm);
}

void controlLoop()
{
  if (timer_fired)
  {

    timer_time = micros();
    timer_fired = false;
    // Read from serial
    readSerial();

    if (dFunction)
      return;

    // Control system
    controllerToLED();
    // Performance metrics
    performanceMetrics();
    if (test)
    {
      if ((int)((micros() - time_now) / 1000000.0) % 5 == 0 && (int)((micros() - time_now) / 1000000.0) % 10 != 0)
      {
        r = calculateLux2Voltage(30);
      }
      if ((int)((micros() - time_now) / 1000000.0) % 10 == 0)
      {
        r = calculateLux2Voltage(3);
      }
    }
    // String dataString = "r: " + String(r) + ", lux: " + String(lux);
    // Serial.println(dataString);
    if (visualize)
    {
      Serial.print(calculateLux(analogRead(ADC_PIN)));
      Serial.print(" ");
      Serial.print(calculateVoltage2Lux(r));
      Serial.print(" ");
      Serial.print(max(0, calculateVoltage2Lux(volt) - calculateVoltage2Lux(gain * my_pid.getDutyCycle() * 4095)));
      Serial.print(" ");
      Serial.print(my_pid.getDutyCycle()*100);
      Serial.print(" ");
      Serial.println("0 40");
      //Serial.println((micros() - time_now) / 1000000.0);
    }

    if (streamL)
    {
      Serial.printf("s l %d %f %d\n", LUMINAIRE, calculateVoltage2Lux(volt), millis());
    }
    if (streamD)
    {
      Serial.printf("s d %d %f %d\n", LUMINAIRE, my_pid.getDutyCycle(), millis());
    }

    counter++;
    sumVolt = 0;
    countVolt = 0;
  }
  else
  {
    int read_adc = analogRead(ADC_PIN);
    float voltTemp = calculateVoltage(read_adc);
    sumVolt += voltTemp;
    countVolt++;
  }
}

void performanceMetrics()
{
  E += my_pid.getDutyCycle() * ((float)0.1 / 1000); // Energy (without considering the power factor)

  V += max(0, calculateVoltage2Lux(r) - calculateVoltage2Lux(volt)); // Luminance error (no-mean value)

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

float calculateLux2Voltage(float lux)
{
  float resistance = pow(10, (log10(lux) * m + b));
  return vcc * R / (resistance + R);
}

float calculateVoltage2Lux(float voltage)
{
  float resistance = (vcc * R) / voltage - R;
  return pow(10, (log10(resistance) - b) / m);
}

float calculateVoltage(float read_adc)
{
  return read_adc / (DAC_RANGE / vcc); // ADC to voltage
}

float calculateLDR(float read_adc)
{
  return (vcc * R) / calculateVoltage(read_adc) - R;
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

void sendMessage()
{
  if (millis() >= time_to_write)
  {
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;
    unsigned long div = counterTx * 10;
    for (int i = 0; i < 8; i++)
      canMsgTx.data[7 - i] = '0' + ((div /= 10) % 10);
    err = can0.sendMessage(&canMsgTx);

    Serial.print("Sending message ");
    Serial.print(counterTx);
    Serial.print(" from node ");
    Serial.println(node_address, HEX);
    counterTx++;
    time_to_write = millis() + write_delay;
  }
}

void receiveMessage()
{
  if (data_available)
  {
    can0.readMessage(&canMsgRx);
    Serial.print("Received message number ");
    Serial.print(counterRx++);
    Serial.print(" from node ");
    Serial.print(canMsgRx.can_id, HEX);
    Serial.print(" : ");
    for (int i = 0; i < canMsgRx.can_dlc; i++)
      Serial.print((char)canMsgRx.data[i]);
    Serial.println(" ");
    data_available = false;
  }
}
