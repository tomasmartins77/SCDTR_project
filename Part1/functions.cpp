#include "functions.h"
#include <Arduino.h>

functions::functions()
{
    // Constructor
}

float functions::calculateLux2Voltage(float lux) // calculate lux to voltage
{
    float resistance = calculateLux2LDR(lux);
    return vcc * R / (resistance + R);
}

float functions::calculateLux2LDR(float lux) {
    return pow(10, (log10(lux) * m + b));
}

float functions::calculateLux2adc(float lux) {
    float voltage = calculateLux2Voltage(lux);
    return voltage * DAC_RANGE / vcc;
}

float functions::calculateVoltage2Lux(float voltage) // calculate voltage to lux
{
    float resistance = (vcc * R) / voltage - R;
    return pow(10, (log10(resistance) - b) / m);
}

float functions::calculateVoltage(float read_adc)
{
    return read_adc * vcc / DAC_RANGE; // ADC to voltage
}

float functions::calculateLDR(float read_adc) // Calculate the resistance of the LDR from adc
{
    return (vcc * R) / calculateVoltage(read_adc) - R;
}

float functions::calculateLux(float read_adc)
{
    return pow(10, (log10(calculateLDR(read_adc)) - b) / m); // resistance to lux
}

float functions::calculateTau(float LDR) // Calculate the time constant theoretically
{
    return (LDR * R) / (LDR + R) * C;
}

void functions::addToBuffer(float value1, float value2) // Add to the buffer
{
    last_minute_buffer[head].l = value1;
    last_minute_buffer[head].d = value2;
    head = (head + 1) % bufferSize;
}

int functions::getBufferSize() // Get the buffer size
{
    return bufferSize;
}