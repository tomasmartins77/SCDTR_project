void interface(const char *buffer)
{
    char command, secondCommand, x;
    int luminaire;
    float value;

    command = buffer[0];

    switch (command)
    {
    case 'u':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0 && value <= 1)
        {
            test = value; // test
            time_now = micros();
            r = functions.calculateLux2Voltage(3);
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'd':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0 && value <= 1)
        {
            dFunction = true;           // d function
            my_pid.setDutyCycle(value); // duty cycle
            analogWrite(LED_PIN, value * DAC_RANGE);
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'r':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            dFunction = false;
            r = functions.calculateLux2Voltage(value); // reference
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'a':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setAntiWindup(value); // anti-windup
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'o':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setOccupancy(value); // occupancy
            if (my_pid.getOccupancy())
            {
                r = functions.calculateLux2Voltage(occupancy_person);
            }
            else
            {
                r = functions.calculateLux2Voltage(occupancy_no_person);
            }
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'k':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            my_pid.setFeedback(value); // feedback
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'B':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setB(value);
            my_pid.setK(1 / (gain * my_pid.getB()));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'K':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setK(value);
            my_pid.setB(1 / (gain * my_pid.getK()));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'I':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setTi(value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'T':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && value >= 0)
        {
            my_pid.setTt(value);
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
    case 'v':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire && (value == 0 || value == 1))
        {
            visualize = (int)value; // visualize
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'z': // restart
        sscanf(buffer, "%c %d", &command, &luminaire);
        if (LUMINAIRE == luminaire)
        {
            Serial.println("ack");
            time_now = micros();
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'R':
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
    case 'g':
        sscanf(buffer, "%c %c %d", &command, &secondCommand, &luminaire); // get
        switch (secondCommand)
        {
        case 'd':
            if (LUMINAIRE == luminaire)
            {
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
                Serial.printf("r %d %f\n", luminaire, functions.calculateVoltage2Lux(r)); // reference
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'l':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("l %d %f\n", luminaire, functions.calculateVoltage2Lux(volt)); // lux
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
                Serial.printf("x %d %lf\n", LUMINAIRE, max(0, functions.calculateVoltage2Lux(volt) - functions.calculateVoltage2Lux(gain * my_pid.getDutyCycle() * 4095))); // external luminance
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
                Serial.printf("t %d %f\n", luminaire, (micros() - time_now) / 1000000.0); // time
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
        case 'B':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("B %d %f\n", luminaire, my_pid.getB());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'K':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("K %d %f\n", luminaire, my_pid.getK());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'I':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Ti %d %f\n", luminaire, my_pid.getTi());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'T':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Tt %d %f\n", luminaire, my_pid.getTt());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'g':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("g %d %f\n", luminaire, gain);
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'R':
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