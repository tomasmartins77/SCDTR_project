#include "pid.h"

const int LUMINAIRE = 1;

// luminaire constants
const int LED_PIN = 15;
const int ADC_PIN = A0;
const int DAC_RANGE = 4095;
const float vcc = 3.3;
const int sampInterval = 10; // 100hZ OR 0.01s
const int conv = DAC_RANGE / vcc;
const float m = -0.9;
const float R = 10000;
const float b = log10(300000) + 0.8;

// PID constants
pid my_pid{0, 10, 100, 100}; // K, b, Ti, Tt, Td, N
float r{1.0};

// luminaire variables
float occupancy_person = 7;
float occupancy_no_person = 1;
float set_duty_cycle;
float lux;
int lux_counter;
float lux_sum;
float E;
float V;
float fk;
float powerMax = 1;
int counter;
float gain;
float duty_cycle_k1;
float duty_cycle_k2;

unsigned long previousTime = 0;

void setup()
{ // the setup function runs once
  Serial.begin(115200);
  analogReadResolution(12);    // default is 10
  analogWriteFreq(60000);      // 60KHz, about max
  analogWriteRange(DAC_RANGE); // 100% duty cycle
  calibrate();
}

void calibrate()
{
  analogWrite(LED_PIN, 0);
  delay(2000);

  float lux1 = calculate_adc_lux(analogRead(ADC_PIN));

  analogWrite(LED_PIN, 2000);
  delay(2000);

  float lux2 = calculate_adc_lux(analogRead(ADC_PIN));

  gain = (lux2 - lux1) / (0.4884 - 0);
  my_pid.setK(gain);
}

void Read_from_Serial()
{
  static String buffer;

  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n')
    {
      interface(buffer.c_str());
      buffer = "";
    }
    else
    {
      buffer += c;
    }
  }
}

void loop()
{ // the loop function runs cyclically
  int u;
  unsigned long currentTime = millis();

  // Compute step and return if less than sampling period
  unsigned long h = (float)(currentTime - previousTime);
  if (h >= sampInterval)
  {
    // Read from serial
    Read_from_Serial();
    // Digital filter
    digitalFilter();
    // Calculate lux average
    lux = lux_sum / lux_counter;
    // Control system
    if (my_pid.getFeedback())
    {
      u = (int)my_pid.compute_control(r, lux, h);
      // write the value to the LED
      analogWrite(LED_PIN, u);
      my_pid.setDutyCycle((float)u / DAC_RANGE);
    }
    else
    {
      analogWrite(LED_PIN, set_duty_cycle * DAC_RANGE);
    }

    // String dataString = "u: " + String(u) + ", r: " + String(r) + ", lux: " + String(lux) + ", lux_counter: " + String(lux_counter);
    //  Serial.println(dataString);

    // Serial.print(lux);
    // Serial.print(" ");
    // Serial.print(lux - gain * my_pid.getDutyCycle()); // external luminance errado
    // Serial.print(" ");
    // Serial.print(r);
    // Serial.print(" ");
    // Serial.println(" 0 24 ");

    // Performance metrics
    performanceMetrics(h);

    counter++;
    previousTime = currentTime;
    lux_counter = 0;
    lux_sum = 0;
  }
  else
  {
    digitalFilter();
  }
}

void performanceMetrics(unsigned long h)
{
  E += my_pid.getDutyCycle() * (h / 1000);

  V += max(0, r - lux);

  if (counter > 1)
  {
    if ((my_pid.getDutyCycle() - duty_cycle_k1) * (duty_cycle_k1 - duty_cycle_k2) < 0)
    {
      fk += abs(my_pid.getDutyCycle() - duty_cycle_k1) + abs(duty_cycle_k1 - duty_cycle_k2);
    }
    duty_cycle_k2 = duty_cycle_k1;
    duty_cycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 1)
  {
    duty_cycle_k1 = my_pid.getDutyCycle();
  }
  else if (counter == 0)
  {
    duty_cycle_k2 = my_pid.getDutyCycle();
  }
}

void digitalFilter()
{
  // read the value from the ADC
  int read_adc = analogRead(ADC_PIN);
  // transform the ADC into lux
  float templux = calculate_adc_lux(read_adc);
  lux_counter++;
  lux_sum += templux;
}

float calculate_adc_lux(float read_adc)
{
  float voltage = float(read_adc) / conv;
  float ldr = (vcc * R) / voltage - R;
  return pow(10, (log10(ldr) - b) / m);
}

void interface(const char *buffer)
{
  char command, secondCommand;
  int luminaire;
  float value;

  command = buffer[0];

  switch (command)
  {
  case 'd':
    sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
    if (LUMINAIRE == luminaire)
    {
      set_duty_cycle = value;
      analogWrite(LED_PIN, set_duty_cycle * DAC_RANGE); // duty cycle
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
      set_duty_cycle = my_pid.getDutyCycle();
      r = calculate_adc_lux(my_pid.getDutyCycle() * DAC_RANGE);
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
  case 'S':
    break;
  case 's':
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
          Serial.printf("d %d %f\n", luminaire, set_duty_cycle);
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
      if (LUMINAIRE == luminaire)
      {
        // Serial.printf("a %d %f\n", luminaire, my_pid.getAntiWindup());
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
