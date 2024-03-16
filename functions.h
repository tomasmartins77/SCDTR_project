// LuxSensor.h
#ifndef functions_h
#define functions_h

class functions
{
public:
    functions();
    float calculateLux2Voltage(float lux);
    float calculateLux2LDR(float lux);
    float calculateLux2adc(float lux);
    float calculateVoltage2Lux(float voltage);
    float calculateVoltage(float read_adc);
    float calculateLDR(float read_adc);
    float calculateLux(float read_adc);
    float calculateTau(float LDR);
    void addToBuffer(float value1, float value2);
    int getBufferSize();
    struct DataPoint
    { // Structure to store the data
        float l;
        float d;
    };

    DataPoint last_minute_buffer[6000]; // Buffer to store the data

private:
    const float vcc = 3.3;        // Maximum voltage
    const float m = -0.9;         // slope of the Lux-duty cycle curve
    const float R = 10000;        // resistance of the LDR circuit
    const float C = 10e-6;        // capacitance of the LDR circuit
    const float b = 6.2522;       // intercept of the Lux-duty cycle curve
    const float DAC_RANGE = 4095; // Range of the DAC
    const int bufferSize = 6000;  // Size of the buffer
    int head = 0;                 // Index of the first empty position in the buffer
};

#endif