#include "pid.h"

const int LUMINAIRE = 1;

// luminaire constants
const int LED_PIN = 15;              // Pin that connects to the LED
const int ADC_PIN = A0;              // Pin that connects to the LDR
const int DAC_RANGE = 4095;          // Range of the DAC
const float vcc = 3.3;               // Maximum voltage
const int sampInterval = 10;         // 100hZ OR 0.01s
const float m = -0.8;                // slope of the Lux-duty cycle curve
const float R = 10000;               // resistance of the LDR circuit
const float b = log10(225000) + 0.8; // intercept of the Lux-duty cycle curve

// PID constants
pid my_pid{15, 10, 28, 10}; // K, b, Ti, Tt, Td, N
float r{1.0};                // reference

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
    Serial.print(my_pid.getDutyCycle()); // external luminance
    Serial.print(" ");
    Serial.print(r);
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

float calculateLux(float read_adc)
{
  float voltage = float(read_adc) / (DAC_RANGE / vcc); // ADC to voltage
  float ldr = (vcc * R) / voltage - R;                 // voltage to resistance
  return pow(10, (log10(ldr) - b) / m);                // resistance to lux
}

void addToBuffer(float value1, float value2)
{
  last_minute_buffer[head].l = value1;
  last_minute_buffer[head].d = value2;
  head = (head + 1) % bufferSize;
}

void interface(const char *buffer)
{
  char command, secondCommand, x;
  int luminaire;
  float value;

  command = buffer[0];

  switch (command)
  {
  case 'd':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      dutyCycle_const = value;
      analogWrite(LED_PIN, dutyCycle_const * DAC_RANGE); // duty cycle
      Serial.println("ack");
    }
    else
    {
      Serial.println("err");
    }
    break;
  case 'r':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      r = value; // reference
      Serial.println("ack");
    }
    else
      Serial.println("err");
    break;
  case 'a':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      my_pid.setAntiWindup(value); // anti-windup
      Serial.println("ack");
    }
    else
      Serial.println("err");
    break;
  case 'o':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      my_pid.setOccupancy(value); // occupancy
      if (my_pid.getOccupancy())
      {
        r = occupancy_person;
      }
      else
      {
        r = occupancy_no_person;
      }
      Serial.println("ack");
    }
    else
      Serial.println("err");
    break;
  case 'k':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      dutyCycle_const = my_pid.getDutyCycle();
      r = calculateLux(my_pid.getDutyCycle() * DAC_RANGE);
      my_pid.setFeedback(value); // feedback
      Serial.println("ack");
    }
    else
      Serial.println("err");
    break;
  case 'B':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      my_pid.setBumplessTransfer(value); // bumpless transfer
      Serial.println("ack");
    }
    else
      Serial.println("err");
    break;
  case 's':
    sscanf(buffer, "%c %c %d %f", &command, &secondCommand, &luminaire);
    switch (secondCommand)
    {
    case 'l':
      if (LUMINAIRE == luminaire)
      {
        streamL = !streamL;
        Serial.println("ack");
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'd':
      if (LUMINAIRE == luminaire)
      {
        streamD = !streamD;
        Serial.println("ack");
      }
      else
      {
        Serial.println("err");
      }
      break;
    }
    break;
  case 'S':
    sscanf(buffer, "%c %c %d %f", &command, &secondCommand, &luminaire);
    switch (secondCommand)
    {
    case 'l':
      if (LUMINAIRE == luminaire)
      {
        streamL = !streamL;
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'd':
      if (LUMINAIRE == luminaire)
      {
        streamD = !streamD;
      }
      else
      {
        Serial.println("err");
      }
      break;
    }
    break;
  case 'g':
    sscanf(buffer, "%c %c %d", &command, &secondCommand, &luminaire); // get
    switch (secondCommand)
    {
    case 'd':
      if (LUMINAIRE == luminaire)
      {
        if (my_pid.getFeedback())
        {
          Serial.printf("d %d %f\n", luminaire, my_pid.getDutyCycle()); // duty cycle
        }
        else
        {
          Serial.printf("d %d %f\n", luminaire, dutyCycle_const);
        }
        Serial.printf("d %d %f\n", luminaire, my_pid.getDutyCycle()); // duty cycle
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'r':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("r %d %f\n", luminaire, r); // reference
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'l':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("l %d %f\n", luminaire, lux); // lux
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'o':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("o %d %f\n", luminaire, my_pid.getOccupancy()); // occupancy
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'a':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("a %d %f\n", luminaire, my_pid.getAntiWindup()); // anti-windup
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'k':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("k %d %f\n", luminaire, my_pid.getFeedback()); // feedback
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'x':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("x %d %lf\n", LUMINAIRE, lux - gain * my_pid.getDutyCycle()); // external luminance
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'p':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("p %d %f\n", luminaire, my_pid.getDutyCycle() * powerMax); // power
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 't':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("t %d %f\n", luminaire, micros() / 1000000.0); // time
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'b':
      sscanf(buffer, "%c %c %c %d", &command, &secondCommand, &x, &luminaire);
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("b %c %i ", x, luminaire);
        for (int i = 0; i < bufferSize; i++)
        {
          if (x == 'l')
          {
            Serial.printf("%f, ", last_minute_buffer[i].l);
          }
          else
          {
            Serial.printf("%f, ", last_minute_buffer[i].d);
          }
        }
        Serial.println();
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'e':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("e %d %f\n", luminaire, E * powerMax);
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'v':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("v %d %f\n", luminaire, V / counter);
      }
      else
      {
        Serial.println("err");
      }
      break;
    case 'f':
      if (LUMINAIRE == luminaire)
      {
        Serial.printf("f %d %f\n", luminaire, fk / counter);
      }
      else
      {
        Serial.println("err");
      }
      break;
    default:
      Serial.println("err");
      return;
    }
    break;
  default:
    Serial.println("err");
    return;
  }
}
