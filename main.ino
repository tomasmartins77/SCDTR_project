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
int viz = 0;
int test = 0;
float time_now;
// PID constants
pid my_pid{0.3}; // K
functions functions;
float r{3.0}; // reference

// luminaire variables
float occupancy_person = 10;   // occupancy reference luminance
float occupancy_no_person = 3; // no occupancy reference luminance
float volt;                    // voltage
float H;                       // conversion
float E;                       // Energy
float V;                       // Luminance error
float fk;                      // Flicker
int counter = 1;               // counter for the performance metrics
float gain;                    // gain of the control system
float dutyCycle_k1;            // duty cycle for the performance metrics
float dutyCycle_k2;            // duty cycle for the performance metrics
bool streamL = false;          // stream luminance
bool streamD = false;          // stream duty cycle
bool dFunction = false;        // disable pid when changing duty cycle
float sumVolt = 0;             // sum of the voltage
int countVolt = 0;             // count of the voltage
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

// Function to initialize system settings and peripherals
void setup()
{
  // Retrieve unique identifier of the microcontroller's flash memory
  flash_get_unique_id(this_pico_flash_id);

  // Set node address using a specific byte of the flash memory ID
  node_address = this_pico_flash_id[6];

  // Start serial communication at baud rate 115200
  Serial.begin(115200);

  // Set analog-to-digital converter (ADC) resolution to 12 bits
  analogReadResolution(12);

  // Set frequency of the analog output (PWM) to 60 kHz
  analogWriteFreq(60000);

  // Set range of the analog output to match the DAC range constant
  analogWriteRange(DAC_RANGE);

  // Perform calibration to adjust system parameters
  calibrate();

  // Add repeating timer with a period of 10 milliseconds (-10 ms delay for immediate execution)
  add_repeating_timer_ms(-10, my_repeating_timer_callback, NULL, &timer);
}

// Function to initialize CAN communication and related settings
void setup1()
{
  // Reset CAN controller
  can0.reset();

  // Set CAN bitrate to 1000 kbps
  can0.setBitrate(CAN_1000KBPS);

  // Set CAN controller to normal mode
  can0.setNormalMode();

  // Enable interrupt for CAN communication on specified pin
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);

  // Set initial time to write CAN messages
  time_to_write = millis() + write_delay;
}

void loop1()
{
  sendMessage();
  receiveMessage();
}

void loop()
{
  controlLoop();
}

// Function to control the LED brightness based on the control system output
void controllerToLED()
{
  int pwm; // Variable to store the PWM value
  float u; // Variable to store the control signal

  // Read the value from the analog-to-digital converter (ADC)
  int read_adc = analogRead(ADC_PIN);

  // Transform the ADC reading into voltage
  float voltTemp = functions.calculateVoltage(read_adc);

  // Calculate the average voltage
  volt = (sumVolt + voltTemp) / (countVolt + 1);

  // Compute the control signal using the PID controller
  u = my_pid.computeControl(r, volt);
  u *= DAC_RANGE;
  // Convert the control signal to PWM value
  pwm = (int)u;

  // Set the duty cycle of the LED based on the control signal
  my_pid.setDutyCycle(u / DAC_RANGE);

  // Add the lux and duty cycle values to the buffer
  functions.addToBuffer(functions.calculateVoltage2Lux(volt), my_pid.getDutyCycle());

  // Write the PWM value to the LED
  analogWrite(LED_PIN, pwm);
}

// Function to control the loop execution based on timer firing
void controlLoop()
{
  if (timer_fired) // If the timer has fired
  {
    timer_fired = false;                     // Reset timer flag
    float timer = micros();                  // Get current time
    float interval = (timer - previousTime); // Calculate time interval

    // Read serial input
    readSerial();

    // If duty cycle adjustment is disabled, return without further processing
    if (dFunction)
      return;

    // Execute control system
    controllerToLED();

    // Calculate performance metrics
    performanceMetrics();

    // Stream luminance or duty cycle to serial monitor if enabled
    if (streamL) // Stream luminance flag
    {
      Serial.printf("s l %d %.2f %.2f\n", LUMINAIRE, functions.calculateVoltage2Lux(volt), micros() / 1000000.0);
    }
    if (streamD) // Stream duty cycle flag
    {
      Serial.printf("s d %d %.2f %.2f\n", LUMINAIRE, my_pid.getDutyCycle(), micros() / 1000000.0);
    }

    // Serial.println(interval - 10000);

    // Reset variables for the next iteration
    counter++;
    sumVolt = 0;
    countVolt = 0;
    previousTime = timer; // Update previous time
  }
  else // If the timer hasn't fired yet
  {
    if (countVolt < 50) // Perform voltage sampling to calculate the average
    {
      int read_adc = analogRead(ADC_PIN);                    // Read ADC value
      float voltTemp = functions.calculateVoltage(read_adc); // Calculate voltage
      sumVolt += voltTemp;                                   // Accumulate voltage sum
      countVolt++;                                           // Increment voltage count
    }
  }
}

// Function to calculate performance metrics
void performanceMetrics()
{
  // Update energy by adding current duty cycle multiplied by the sampling interval
  E += my_pid.getDutyCycle() * ((float)0.01);

  // Update luminance error by subtracting current lux value from reference lux value
  V += max(0, functions.calculateVoltage2Lux(r) - functions.calculateVoltage2Lux(volt));

  // Calculate flicker if enough samples available (at least three)
  if (counter > 2)
  {
    // Check if the duty cycle changes direction (indicates flicker)
    if ((my_pid.getDutyCycle() - dutyCycle_k1) * (dutyCycle_k1 - dutyCycle_k2) < 0)
    {
      // Update flicker by adding absolute differences between current and previous duty cycles
      fk += abs(my_pid.getDutyCycle() - dutyCycle_k1) + abs(dutyCycle_k1 - dutyCycle_k2);
    }
    // Update duty cycle history
    dutyCycle_k2 = dutyCycle_k1;
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 2) // For the second sample, update duty cycle k1
  {
    dutyCycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 1) // For the first sample, update duty cycle k2
  {
    dutyCycle_k2 = my_pid.getDutyCycle();
  }
}

// Function to calibrate the system
void calibrate()
{
  delay(2000);             // Wait for stabilization
  analogWrite(LED_PIN, 0); // Turn off the LED
  delay(2000);             // Wait for stabilization

  // Read initial lux value
  float lux1 = functions.calculateLux(analogRead(ADC_PIN));

  analogWrite(LED_PIN, DAC_RANGE); // Turn on the LED at 100% duty cycle
  delay(2000);                     // Wait for stabilization

  // Read final lux value
  float lux2 = functions.calculateLux(analogRead(ADC_PIN));

  // Calculate the gain of the system
  gain = (lux2 - lux1) / (1 - 0);

  analogWrite(LED_PIN, functions.calculateLux2adc(r)); // Turn off the LED

  lux1 = r;
  // Set the reference lux value
  r = functions.calculateLux2Voltage(r);

  H = r / lux1;
  // Adjust the proportional term of the PID controller based on the system gain
  my_pid.setB(1 / (H * gain * my_pid.getK()));
  float LDR = functions.calculateLux2LDR(lux1);
  // Set the integral time constant (Ti) of the PID controller based on LDR value
  my_pid.setTi(functions.calculateTau(LDR));

  delay(1000); // Wait for stabilization
}

// Function to read and process serial input
void readSerial()
{
  static String buffer; // Static buffer to store incoming serial data

  // Continue reading while there is data available in the serial buffer
  while (Serial.available() > 0)
  {
    char c = Serial.read(); // Read a character from the serial buffer

    if (c == '\n') // If end of line is reached
    {
      interface(buffer.c_str()); // Process the command stored in the buffer
      buffer = "";               // Clear the buffer for the next command
    }
    else
    {
      buffer += c; // Add the character to the buffer
    }
  }
}

// Function to send CAN message
void sendMessage()
{
  // Check if it's time to send a message
  if (millis() >= time_to_write)
  {
    // Set CAN message ID and data length
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;

    // Prepare message data
    unsigned long div = counterTx * 10;
    for (int i = 0; i < 8; i++)
      canMsgTx.data[7 - i] = '0' + ((div /= 10) % 10);

    // Send the message and handle errors
    err = can0.sendMessage(&canMsgTx);
    if (err == MCP2515::ERROR_OK) // Message sent successfully
    {
      Serial.print("Sending message ");
      Serial.print(counterTx);
      Serial.print(" from node ");
      Serial.println(node_address, HEX);
    }
    else if (err == MCP2515::ERROR_FAILTX) // Message transmission failed
    {
      Serial.print("Error sending message: ");
      Serial.println(err);
    }
    else if (err == MCP2515::ERROR_ALLTXBUSY) // All transmission buffers are busy
    {
      Serial.print("Error sending message, all tx busy: ");
      Serial.println(err);
    }

    // Update counters and timing
    counterTx++;
    time_to_write = millis() + write_delay;
  }
}

// Function to receive CAN message
void receiveMessage()
{
  // Check if data is available
  if (data_available)
  {
    // Read the message and handle errors
    err = can0.readMessage(&canMsgRx);
    if (err == MCP2515::ERROR_OK) // Message received successfully
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
    else if (err == MCP2515::ERROR_FAIL) // Message reception failed
    {
      Serial.println("Failed to receive message");
    }

    // Reset data availability flag
    data_available = false;
  }
}
