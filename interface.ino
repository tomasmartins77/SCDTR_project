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
            analogWrite(LED_PIN, value * DAC_RANGE); // duty cycle
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
            my_pid.setB(value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'K':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire)
        {
            my_pid.setK(value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;
    case 'T':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire)
        {
            my_pid.setTi(value);
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
    case 'w':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire)
        {
            linear = (int)value; // linear
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
        break;
    case 'v':
        sscanf(buffer, "%c %d %f", &command, &luminaire, &value);
        if (LUMINAIRE == luminaire)
        {
            visualize = (int)value; // gain
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
                Serial.printf("x %d %lf\n", LUMINAIRE, max(0, lux - gain * my_pid.getDutyCycle())); // external luminance
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
        case 'T':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("Ti %d %f\n", luminaire, my_pid.getTi());
            }
            else
            {
                Serial.println("err");
            }
            break;
        case 'u':
            if (LUMINAIRE == luminaire)
            {
                Serial.printf("u %d %f\n", luminaire, tau);
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