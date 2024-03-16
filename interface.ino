void interface(const char *buffer)
{
    char command, secondCommand, x;
    int luminaire;
    float value;

    command = buffer[0];

    switch (command)
    {
    case 'd': // change the duty cycle of the luminaire i to value (0-1)
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0 && value <= 1)
        {
            dFunction = true;                        // blocks controller
            my_pid.setDutyCycle(value);              // set duty cycle
            analogWrite(LED_PIN, value * DAC_RANGE); // change the LED
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'r': // calculate the reference value for the luminaire i to value (lux)
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            dFunction = false;                         // unblocks controller
            r = functions.calculateLux2Voltage(value); // reference to volts
            H = r / value;                             // volt/lux
            // Calculate Light Dependent Resistor (LDR) value
            float LDR = functions.calculateLux2LDR(value);
            // Set the integral time constant (Ti) of the PID controller based on LDR value
            my_pid.setTi(functions.calculateTau(LDR));
            my_pid.setB(1 / (H * gain * my_pid.getK()));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'a': // activate or deactivate the anti-windup for the luminaire i to value (0-1)
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setAntiWindup(value); // anti-windup
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'o': // activate or deactivate the occupancy for the luminaire i to value (0-1)
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setOccupancy(value); // set occupancy
            if (my_pid.getOccupancy())
            {
                r = functions.calculateLux2Voltage(occupancy_person); // set reference to occupancy
                H = r / occupancy_person;
                // Calculate Light Dependent Resistor (LDR) value
                float LDR = functions.calculateLux2LDR(value);
                // Set the integral time constant (Ti) of the PID controller based on LDR value
                my_pid.setTi(functions.calculateTau(LDR));
                my_pid.setB(1 / (H * gain * my_pid.getK()));
            }
            else
            {
                r = functions.calculateLux2Voltage(occupancy_no_person); // set reference to no occupancy
                H = r / occupancy_no_person;
                // Calculate Light Dependent Resistor (LDR) value
                float LDR = functions.calculateLux2LDR(value);
                // Set the integral time constant (Ti) of the PID controller based on LDR value
                my_pid.setTi(functions.calculateTau(LDR));
                my_pid.setB(1 / (H * gain * my_pid.getK()));
            }
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'k': // activate or deactivate the feedback for the luminaire i to value (0-1)
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setFeedback(value); // feedback
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'B': // change the b value for the luminaire i to value x
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setB(value);
            my_pid.setK(1 / (H * gain * my_pid.getB()));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'K': // change the K value for the luminaire i to value x
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setK(value);
            my_pid.setB(1 / (H * gain * my_pid.getK()));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'I': // change the Ti value for the luminaire i to value x
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setTi(value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'T': // change the Tt value for the luminaire i to value x
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setTt(value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 's': // stream the values of (l or d) of luminaire i
        sscanf(buffer, "%c %c %d %f", &command, &secondCommand, &luminaire);
        switch (secondCommand)
        {
        case 'l':
            if (LUMINAIRE == luminaire)
            {
                streamL = true;
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
                streamD = true;
                Serial.println("ack");
            }
            else
            {
                Serial.println("err");
            }
            break;
        }
        break;
    case 'S': // stop stream the values of (l or d) of luminaire i
        sscanf(buffer, "%c %c %d %f", &command, &secondCommand, &luminaire);
        switch (secondCommand)
        {
        case 'l':
            if (LUMINAIRE == luminaire)
            {
                streamL = false;
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'd':
            if (LUMINAIRE == luminaire)
            {
                streamD = false;
            }
            else
            {
                Serial.println("err");
            }
            break;
        }
    case 'R': // change the bumpless transfer for the luminaire i to value x
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setBumplessTransfer(value);
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'g': // get functions
        sscanf(buffer, "%c %c %d", &command, &secondCommand, &luminaire);
        switch (secondCommand)
        {
        case 'd': // get the duty cycle of the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("d %d %f\n", luminaire, my_pid.getDutyCycle()); // duty cycle
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'r': // get the reference value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("r %d %f\n", luminaire, functions.calculateVoltage2Lux(r)); // reference
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'l': // get the lux value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("l %d %f\n", luminaire, functions.calculateLux(analogRead(ADC_PIN))); // lux
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'o': // get the occupancy value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("o %d %f\n", luminaire, my_pid.getOccupancy()); // occupancy
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'a': // get the anti-windup value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("a %d %f\n", luminaire, my_pid.getAntiWindup()); // anti-windup
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'k': // get the feedback value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("k %d %f\n", luminaire, my_pid.getFeedback()); // feedback
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'x': // get the external luminance value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("x %d %lf\n", LUMINAIRE, abs(functions.calculateVoltage2Lux(volt) - gain * my_pid.getDutyCycle())); // external luminance
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'p': // get the instantaneous power value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                float powerMax = 0.0162;                                                 // Maximum power dissipated (0.29 V of resistor, I = V/R, P = Vled*I = 2.63*6.17*10^-3 = 0.0162 W)
                Serial.printf("p %d %f\n", luminaire, my_pid.getDutyCycle() * powerMax); // power
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 't': // get the time value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("t %d %f\n", luminaire, micros() / 1000000.0); // time
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'b': // get the last minute buffer of the luminaire i
            sscanf(buffer, "%c %c %c %d", &command, &secondCommand, &x, &luminaire);
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("b %c %i ", x, luminaire);
                for (int i = 0; i < functions.getBufferSize(); i++)
                {
                    if (x == 'l')
                    {
                        Serial.printf("%f, ", functions.last_minute_buffer[i].l);
                    }
                    else
                    {
                        Serial.printf("%f, ", functions.last_minute_buffer[i].d);
                    }
                }
                Serial.println();
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'e': // get the energy value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                float powerMax = 0.0162;                                                        // Maximum power dissipated (0.29 V of resistor, I = V/R, P = Vled*I = 2.63*6.17*10^-3 = 0.0162 W)
                Serial.printf("e %d %f\n", luminaire, E * powerMax / (float)counter * 1000000); // mu J
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'v': // get the visibility error for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("v %d %f\n", luminaire, V / counter);
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'f': // get the flicker for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("f %d %f\n", luminaire, fk / counter);
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'B': // get the b value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("B %d %f\n", luminaire, my_pid.getB());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'K': // get the K value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("K %d %f\n", luminaire, my_pid.getK());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'I': // get the Ti value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Ti %d %f\n", luminaire, my_pid.getTi());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'T': // get the Tt value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Tt %d %f\n", luminaire, my_pid.getTt());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'g': // get the gain value for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("g %d %f\n", luminaire, gain);
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'R': // get the bumpless transfer for the luminaire i
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Bumpless %d %f\n", luminaire, my_pid.getBumplessTransfer());
            }
            else
            {
                Serial.println("err");
            }
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