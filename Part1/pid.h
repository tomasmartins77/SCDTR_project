#ifndef PID_H
#define PID_H

class pid
{
    float I, D, K, Ti, Td, Tt, b, y_old, b_old, K_old, N, dutyCycle, h;
    int occupancy, antiWindup, feedback, bumpless;

public:
    explicit pid(float _K = 1000);
    ~pid(){};
    float computeControl(float r, float y);
    float saturate(float value, float min_val, float max_val);
    void setK(float newK);
    float getK();
    void setB(float newB);
    float getB();
    void setTi(float newB);
    float getTi();
    void setTt(float newTt);
    float getTt();
    void setAntiWindup(float value);
    float getAntiWindup();
    void setOccupancy(float value);
    float getOccupancy();
    void setFeedback(float value);
    float getFeedback();
    void setDutyCycle(float value);
    float getDutyCycle();
    void setBumplessTransfer(float value);
    float getBumplessTransfer();
};

#endif // PID_H