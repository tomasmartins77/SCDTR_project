#include "pid.h"
#include <iostream>

pid::pid(float _K, float b_, float Ti_, float Tt_, float Td_, float N_)
    : K{_K}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, y_old{0.0},
      occupancy{0}, feedback{1}, antiWindup{1},
      K_old{_K}, b_old{b_}, Tt{Tt_}, dutyCycle{0.0}
{
}

float pid::computeControl(float r, float y, float h, float gain)
{
  float u, v;
  if (feedback)
  {
    float e = r - y;

    I += K_old * (b_old * r - y) - K * (b * r - y);
    b_old = b;
    K_old = K;

    float P = K * (b * r - y);

    float bi = K * h / Ti;
    float ad = Td / (Td + N * h);
    float bd = Td * K * N / (Td + N * h);
    float ao = h / Tt;

    D = ad * D - bd * (y - y_old); // not useful for this project, Td = 0

    v = P + I + D;

    u = saturate(v, float(0), float(4095));

    if (antiWindup)
    {
      I += bi * e + ao * (u - v);
    }
    else
    {
      I += K * h / Ti * e;
    }

    y_old = y;
  }
  else
  {
    v = r / gain * 4095;
    u = saturate(v, float(0), float(4095));
  }

  return u;
}

float pid::saturate(float value, float min_val, float max_val)
{
  if (value < min_val)
  {
    return min_val;
  }
  else if (value > max_val)
  {
    return max_val;
  }
  else
  {
    return value;
  }
}

void pid::setK(float newK)
{
  K = newK;
}

float pid::getK()
{
  return K;
}

void pid::setB(float newB)
{
  b = newB;
}

float pid::getB()
{
  return b;
}

void pid::setTi(float newTi)
{
  Ti = newTi;
}

float pid::getTi()
{
  return Ti;
}

void pid::setAntiWindup(float value)
{
  antiWindup = value;
}

float pid::getAntiWindup()
{
  return antiWindup;
}

void pid::setOccupancy(float value)
{
  occupancy = value;
}

float pid::getOccupancy()
{
  return occupancy;
}

void pid::setFeedback(float value)
{
  feedback = value;
}

float pid::getFeedback()
{
  return feedback;
}

void pid::setDutyCycle(float value)
{
  dutyCycle = value;
}

float pid::getDutyCycle()
{
  return dutyCycle;
}