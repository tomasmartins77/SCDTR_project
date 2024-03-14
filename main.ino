#include "pid.h"
#include "mcp2515.h"
#include <hardware/flash.h>
#include <Arduino.h>
#include <FreeRTOS.h>
#include "functions.h"
#include <task.h>

const int LUMINAIRE = 1;

// luminaire constants
const int LED_PIN = 15;       // Pin that connects to the LED
const int ADC_PIN = A0;       // Pin that connects to the LDR
const float DAC_RANGE = 4095; // Range of the DAC
const float vcc = 3.3;        // Maximum voltage
const int sampInterval = 10;  // 100hZ OR 0.01s

// PID constants
pid my_pid{1000, 0.1, 0.1, 0.1}; // K, b, Ti, Tt, Td, N
functions functions;
float r{3.0}; // reference

// luminaire variables
float occupancy_person = 10;   // occupancy reference luminance
float occupancy_no_person = 3; // no occupancy reference luminance
float volt;                    // voltage
float E;                       // Energy
float V;                       // Luminance error
float fk;                      // Flicker
float powerMax = 0.0162;       // Maximum power dissipated (0.29 V of resistor, I = V/R, P = Vled*I = 2.63*6.17*10^-3 = 0.0162 W)
int counter = 1;               // counter for the performance metrics
float gain;                    // gain of the control system
float dutyCycle_k1;            // duty cycle for the performance metrics
float dutyCycle_k2;            // duty cycle for the performance metrics
bool streamL = false;          // stream luminance
bool streamD = false;          // stream duty cycle
unsigned long time_now = 0;    // time now timer restart
bool dFunction = false;        // disable pid when changing duty cycle
float sumVolt = 0;             // sum of the voltage
int countVolt = 0;             // count of the voltage
int visualize = 0;
unsigned long previousTime = 0;

volatile bool timer_fired{false};
struct repeating_timer timer;

MCP2515 can0{spi0, 17, 19, 16, 18, 10000000}; // channel spi0 CSn17, SPIO TX19, SPIO RX16, SPIO SCK18, 10MHz
uint8_t this_pico_flash_id[8], node_address;  // flash id and node address
struct can_frame canMsgTx, canMsgRx;          // can frames
unsigned long counterTx{0}, counterRx{0};     // counters
MCP2515::ERROR err;                           // error
unsigned long time_to_write;                  // time to write
unsigned long write_delay{1000};              // write delay
const byte interruptPin{20};                  // interrupt pin
volatile byte data_available{false};          // data available

float test = 0;

bool my_repeating_timer_callback(struct repeating_timer *t) // interrupt timer callback for pid
{
  if (!timer_fired)
    timer_fired = true;
  return true;
}

void read_interrupt(uint gpio, uint32_t events) // interrupt callback for can
{
  data_available = true;
}

void setup()
{
  flash_get_unique_id(this_pico_flash_id); // get the flash id
  node_address = this_pico_flash_id[6];    // node address
  Serial.begin(115200);                    // start the serial port
  analogReadResolution(12);                // default is 10
  analogWriteFreq(60000);                  // 60KHz, about max
  analogWriteRange(DAC_RANGE);             // 100% duty cycle
  calibrate();                             // calibrate the gain

  add_repeating_timer_ms(-10, my_repeating_timer_callback, NULL, &timer); // 100 Hz
}

void setup1()
{
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt); // interrupt for can
  time_to_write = millis() + write_delay;
}

void loop1()
{
  receiveMessage();
  sendMessage();
}

void calibrate()
{
  delay(2000);
  analogWrite(LED_PIN, 0); // turn off the LED
  delay(2000);

  float volt1 = functions.calculateVoltage(analogRead(ADC_PIN)); // read the lux

  analogWrite(LED_PIN, DAC_RANGE); // turn on the LED at 100% duty cycle
  delay(2000);

  float volt2 = functions.calculateVoltage(analogRead(ADC_PIN)); // read the lux

  gain = (volt2 - volt1) / (4095 - 0);   // calculate the gain
  r = functions.calculateLux2Voltage(r); // set the reference
  my_pid.setB(0.9 * (1 / (gain * my_pid.getK())));
  analogWrite(LED_PIN, 0); // turn off the LED
  delay(1000);
}

void loop()
{
  controlLoop();
}

void controllerToLED()
{
  int pwm;
  float u;

  // read the value from the ADC
  int read_adc = analogRead(ADC_PIN);

  // transform the ADC into lux
  float voltTemp = functions.calculateVoltage(read_adc);
  volt = (sumVolt + voltTemp) / (countVolt + 1);
  float LDR = functions.calculateLDR(read_adc);

  my_pid.setTi(functions.calculateTau(LDR));

  // Control system
  u = my_pid.computeControl(r, volt);
  pwm = (int)u;
  // set the duty cycle
  my_pid.setDutyCycle(u / DAC_RANGE);
  // Add to the buffer
  functions.addToBuffer(functions.calculateVoltage2Lux(volt), my_pid.getDutyCycle());
  // write the value to the LED
  analogWrite(LED_PIN, pwm);
}

void controlLoop()
{
  if (timer_fired)
  {
    timer_fired = false;
    float timer = micros();
    float interval = (timer - previousTime);

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
        r = functions.calculateLux2Voltage(30);
      }
      if ((int)((micros() - time_now) / 1000000.0) % 10 == 0)
      {
        r = functions.calculateLux2Voltage(3);
      }
    }
    // String dataString = "r: " + String(r) + ", lux: " + String(lux);
    // Serial.println(dataString);
    if (visualize)
    {
      Serial.print(functions.calculateLux(analogRead(ADC_PIN)));
      Serial.print(" ");
      Serial.print(functions.calculateVoltage2Lux(r));
      Serial.print(" ");
      Serial.print(max(0, functions.calculateVoltage2Lux(volt) - functions.calculateVoltage2Lux(gain * my_pid.getDutyCycle() * 4095)));
      Serial.print(" ");
      Serial.print(my_pid.getDutyCycle());
      Serial.print(" ");
      Serial.print(abs(10000 - interval));
      Serial.print(" ");
      // Serial.println("0 40");
      Serial.println((micros() - time_now) / 1000000.0);
    }

    if (streamL)
    {
      Serial.printf("s l %d %f %d\n", LUMINAIRE, functions.calculateVoltage2Lux(volt), millis());
    }
    if (streamD)
    {
      Serial.printf("s d %d %f %d\n", LUMINAIRE, my_pid.getDutyCycle(), millis());
    }
    counter++;
    sumVolt = 0;
    countVolt = 0;
    previousTime = timer;
  }
  else
  {
    if (countVolt < 50)
    {
      int read_adc = analogRead(ADC_PIN);
      float voltTemp = functions.calculateVoltage(read_adc);
      sumVolt += voltTemp;
      countVolt++;
    }
  }
}

void performanceMetrics()
{
  E += my_pid.getDutyCycle() * ((float)0.01); // Energy (without considering the power factor)

  V += max(0, functions.calculateVoltage2Lux(r) - functions.calculateVoltage2Lux(volt)); // Luminance error (no-mean value)

  if (counter > 2) // We need at least two samples to calculate the flicker
  {
    if ((my_pid.getDutyCycle() - dutyCycle_k1) * (dutyCycle_k1 - dutyCycle_k2) < 0)
    {
      fk += abs(my_pid.getDutyCycle() - dutyCycle_k1) + abs(dutyCycle_k1 - dutyCycle_k2); // Flicker
    }
    dutyCycle_k2 = dutyCycle_k1;
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 2)
  {
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 1)
  {
    dutyCycle_k2 = my_pid.getDutyCycle();
  }
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
    if (err == MCP2515::ERROR_OK) // message sent
    {
      Serial.print("Sending message ");
      Serial.print(counterTx);
      Serial.print(" from node ");
      Serial.println(node_address, HEX);
    }
    else if (err == MCP2515::ERROR_FAILTX) // message failed
    {
      Serial.print("Error sending message: ");
      Serial.println(err);
    }
    else if (err == MCP2515::ERROR_ALLTXBUSY) // all tx busy
    {
      Serial.print("Error sending message, all tx busy: ");
      Serial.println(err);
    }

    counterTx++;
    time_to_write = millis() + write_delay;
  }
}

void receiveMessage()
{
  if (data_available)
  {
    err = can0.readMessage(&canMsgRx);
    if (err == MCP2515::ERROR_OK)
    {
      Serial.print("Received message number ");
      Serial.print(counterRx++);
      Serial.print(" from node ");
      Serial.print(canMsgRx.can_id, HEX);
      Serial.print(" : ");
      for (int i = 0; i < canMsgRx.can_dlc; i++)
        Serial.print((char)canMsgRx.data[i]);
      Serial.println(" ");
    }
    else if (err == MCP2515::ERROR_FAIL) // message failed
    {
      Serial.println("Failed to receive message");
    }

    data_available = false;
  }
}
