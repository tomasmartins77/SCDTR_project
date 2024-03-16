#include "pid.h"
#include <iostream>

pid::pid(float _K)
    : K{_K}, b{0.1}, Ti{0.1}, Td{0},
      N{10}, I{0.0}, D{0.0}, y_old{0.0}, h{0.01},
      occupancy{0}, feedback{1}, antiWindup{1}, bumpless{1},
      K_old{_K}, b_old{0.1}, Tt{0.1}, dutyCycle{0.0}
{
}

float pid::computeControl(float r, float y)
{
  float uff, ufb, vfb, u, v;

  uff = r * b * K; // feedforward control
  uff = saturate(uff, float(0), float(1));
  if (!feedback)
    return uff; // feedforward control only

  if (bumpless) // bumpless transfer
  {
    I += K_old * (b_old * r - y) - K * (b * r - y);
  }

  b_old = b;
  K_old = K;

  float P = K * (-y); // proportional control

  float bi = K * h / Ti;
  float ad = Td / (Td + N * h);
  float bd = Td * K * N / (Td + N * h);
  float ao = h / Tt;

  D = ad * D - bd * (y - y_old); // not useful for this project, Td = 0

  ufb = P + I + D; // feedback control

  v = uff + ufb; // total control

  u = saturate(v, float(0), float(1)); // control signal

  float e = r - y; // error

  I += bi * e; // integral control

  if (antiWindup)
  {
    I += ao * (u - v); // integral control with anti-windup
  }

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

void pid::setTt(float value)
{
  Tt = value;
}

float pid::getTt()
{
  return Tt;
}

void pid::setBumplessTransfer(float value)
{
  bumpless = value;
}

float pid::getBumplessTransfer()
{
  return bumpless;
}