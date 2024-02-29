#include "pid.h"
#include <iostream>

pid::pid(float _K, float b_, float Ti_, float Tt_, float Td_, float N_)
    : K{_K}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, y_old{0.0},
      occupancy{0}, feedback{1}, antiWindup{1}, bumplessTransfer{1},
      K_old{_K}, b_old{b_}, Tt{Tt_}, dutyCycle{0.0}
{
}

float pid::computeControl(float r, float y, float h)
{
  float e = r - y;

  if (bumplessTransfer)
  {
    I += K_old * (b_old * e) - K * (b * e);
    b_old = b;
    K_old = K;
  }

  float P = K * (b * r - y);

  float bi = K * h / Ti;
  float ad = Td / (Td + N * h);
  float bd = Td * K * N / (Td + N * h);
  float ao = h / Tt;

  D = ad * D - bd * (y - y_old); // not useful for this project, Td = 0

  float v = P + I + D;

  float u = saturate(v, float(0), float(4095));

  if (antiWindup)
  {
    I += bi * e + ao * (u - v);
  }
  else
  {
    I += K * h / Ti * e;
  }

  //I = saturate(I, float(0), float(4095));

  y_old = y;

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

void pid::setBumplessTransfer(float value)
{
  bumplessTransfer = value;
}

float pid::getBumplessTransfer()
{
  return bumplessTransfer;
}