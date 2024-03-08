#include "pid.h"
#include "mcp2515.h"
#include <hardware/flash.h>
#include <Arduino.h>

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
pid my_pid{5, 12.5, 20, 5}; // K, b, Ti, Tt, Td, N  5, 15, 20, 5
float r{1.0};               // reference

// luminaire variables
float occupancy_person = 7;    // occupancy reference luminance
float occupancy_no_person = 1; // no occupancy reference luminance
float lux;                     // luminance
int lux_counter;               // counter for the digital filter
float lux_sum;                 // sum for the digital filter
float E;                       // Energy
float V;                       // Luminance error
float fk;                      // Flicker
float powerMax = 0.1271;       // Maximum power dissipated (0.108 of the LED and 0.0191 of the resistor)
int counter;                   // counter for the performance metrics
float gain;                    // gain of the control system
float tau;
float dutyCycle_k1;             // duty cycle for the performance metrics
float dutyCycle_k2;             // duty cycle for the performance metrics
bool streamL = false;           // stream luminance
bool streamD = false;           // stream duty cycle
unsigned long previousTime = 0; // will store last time LED was updated
int linear = 0;
int counter_led = 0;
int visualize = 0;
unsigned long time_now = 0;

struct DataPoint
{ // Structure to store the data
  float l;
  float d;
};
const int bufferSize = 5;                 // Size of the buffer
DataPoint last_minute_buffer[bufferSize]; // Buffer to store the data
int head = 0;                             // Index of the first empty position in the buffer

MCP2515 can0{spi0, 17, 19, 16, 18, 10000000}; // channel spi0 or spi1
uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx{0}, counterRx{0};
MCP2515::ERROR err;
unsigned long time_to_write;
unsigned long write_delay{1000};
const byte interruptPin{20};
volatile byte data_available{false};

void read_interrupt(uint gpio, uint32_t events)
{
  data_available = true;
}

void setup()
{
  flash_get_unique_id(this_pico_flash_id);
  node_address = this_pico_flash_id[7];
  Serial.begin(115200);
  analogReadResolution(12);    // default is 10
  analogWriteFreq(60000);      // 60KHz, about max
  analogWriteRange(DAC_RANGE); // 100% duty cycle
  calibrate();                 // calibrate the gain

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

  float lux1 = calculateLux(analogRead(ADC_PIN)); // read the lux

  analogWrite(LED_PIN, DAC_RANGE); // turn on the LED at 100% duty cycle
  delay(3000);

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
  controlLoop();
  // receiveMessage();
  // sendMessage();
}

unsigned long resetVariables(unsigned long currentTime)
{
  previousTime = currentTime;
  lux_counter = 0;
  lux_sum = 0;
  if (counter_led >= 4095)
  {
    counter_led = 0;
  }
  return currentTime;
}

void controllerToLED(unsigned long h)
{
  int pwm;
  float u;

  // Read from serial
  readSerial();
  // Digital filter
  digitalFilter();
  // Calculate lux average
  lux = lux_sum / lux_counter;
  // Control system
  u = my_pid.computeControl(r, lux, h, gain);
  pwm = (int)u;

  if (linear)
  {
    if (counter_led < 2048)
    {
      analogWrite(LED_PIN, 0);
    }
    else
    {
      analogWrite(LED_PIN, DAC_RANGE);
    }
  }
  // set the duty cycle
  my_pid.setDutyCycle(u / DAC_RANGE);
  // write the value to the LED
  analogWrite(LED_PIN, pwm);
  // Add to the buffer
  addToBuffer(lux, my_pid.getDutyCycle());
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
  float LDR = calculateLDR(read_adc);
  tau = (LDR * R) / (LDR + R) * C;
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

void controlLoop()
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
    if (visualize)
    {
      Serial.print(lux);
      Serial.print(" ");
      Serial.print(r);
      Serial.print(" ");
      Serial.print(my_pid.getDutyCycle());
      Serial.print(" ");
      Serial.print(lux - gain * my_pid.getDutyCycle());
      Serial.print(" ");
      // Serial.println("0 25");
      Serial.println(micros() - time_now);
    }

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
    counter_led += 5;
  }
  else
  {
    digitalFilter(); // store values bewteen samples for better accuracy
  }
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
