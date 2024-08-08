////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////
#define PIN_HOME_SW 8
////////////////////////////////////////////////////
// OUTPUTS
////////////////////////////////////////////////////

// MOTOR CONTROL BOARD
#define PWM_PIN 11                 // Used for controlling the motor driver board
const int PWM_FREQUENCY_HZ = 2000; // Desired PWM frequency in Hz
#define DIR_PIN 3                  // LOW FOR POSITIVE, HIGH FOR NEGATIVE, output for an input to the motor driver board

// PWM DRIVERS FOR SOLENOID STOPPER
#define PIN_SOL_STOPPER 6

// SHIFTER
#define PIN_SHIFT_UP 15
#define PIN_SHIFT_DOWN 16

class RadGear
{
public:
    enum Modes : int32_t //
    {
        ABORTING = -3,
        KILLED = -2,
        ERROR = -1,
        INACTIVE = 0,
        RESETTING = 50,
        IDLE = 100,
        SHIFTING = 200, // while shifting, the controller is active
                        // SUPER_SHIFT_UP = 300,
                        // SHIFT_DOWN = 400,
                        // SUPER_SHIFT_DOWN = 500,
    };
};

void setup()
{
    Serial.begin(115200);

    // initialize solenoid stopper
    pinMode(PIN_SOL_STOPPER, OUTPUT);
    // solenoids[Solenoids::STOPPER].init(PIN_SOL_STOPPER, SolenoidTypes::PWM, SolenoidSafety::CONTINUOUS_OK);

    // initialize shifter
    pinMode(PIN_HOME_SW, INPUT);

    // initialize motor controller outputs
    pinMode(DIR_PIN, OUTPUT);

    setupPWM(PWM_PIN, PWM_FREQUENCY_HZ); // initialize motor control pin
}

int mode = RadGear::Modes::INACTIVE;
void loop()
{

    switch (mode)
    {
    case RadGear::Modes::INACTIVE:
        if (digitalRead(PIN_SHIFT_UP))
        {
            digitalWrite(DIR_PIN, LOW);
            mode = RadGear::Modes::RESETTING;
        }
        else if (digitalRead(PIN_SHIFT_DOWN))
        {
            digitalWrite(DIR_PIN, HIGH);
            mode = RadGear::Modes::RESETTING;
        }
        break;

    case RadGear::Modes::RESETTING:
        Serial.println("RESETTING");
        /*
        digitalWrite(PWM_PIN, LOW);
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(PIN_SOL_STOPPER, HIGH);
        delay(300);
        analogWrite(PWM_PIN, 255);
        delay(100);
        digitalWrite(PIN_SOL_STOPPER, LOW);
        analogWrite(PWM_PIN, 75);
        delay(500);
        digitalWrite(PWM_PIN, LOW);
        digitalWrite(PIN_SOL_STOPPER, LOW);
        */
        uint8_t amp = .22 * 255.0;
        // digitalWrite(DIR_PIN, LOW);
        analogWrite(PWM_PIN, amp);
        delay(3000);
        /*
        digitalWrite(DIR_PIN, LOW);
        analogWrite(PWM_PIN, amp);
        delay(1000);
        digitalWrite(DIR_PIN, HIGH);
        analogWrite(PWM_PIN, amp);
        delay(1000);
        */
        analogWrite(PWM_PIN, 0);

        mode = RadGear::Modes::INACTIVE;
        break;

    case RadGear::Modes::IDLE:
        Serial.println("IDLE");
        mode = RadGear::Modes::SHIFTING;
        break;
    case RadGear::Modes::SHIFTING:
        break;

    default:
        break;
    }
}

// SETUP
void setupPWM(int pin, long frequency)
{
    uint8_t prescalerBits = 0;
    long pwmFrequency;

    // Calculate the prescaler and the closest achievable PWM frequency
    for (prescalerBits = 1; prescalerBits <= 7; prescalerBits++)
    {
        pwmFrequency = 1000000 / (long(pow(2, prescalerBits)) * 510);
        if (pwmFrequency < frequency)
        {
            break;
        }
    }

    // Set the prescaler
    switch (prescalerBits)
    {
    case 1: // 1
        TCCR3B = (TCCR3B & 0b11111000) | 0x01;
        break;
    case 2: // 8
        TCCR3B = (TCCR3B & 0b11111000) | 0x02;
        break;
    case 3: // 64
        TCCR3B = (TCCR3B & 0b11111000) | 0x03;
        break;
    case 4: // 256
        TCCR3B = (TCCR3B & 0b11111000) | 0x04;
        break;
    case 5: // 1024
        TCCR3B = (TCCR3B & 0b11111000) | 0x05;
        break;
    }

    // Set the PWM frequency
    pwmFrequency = 1000000 / (long(pow(2, prescalerBits)) * 510);
    long ocrValue = (long(F_CPU) / (long(pow(2, prescalerBits)) * frequency)) - 1;
    if (ocrValue < 65536)
    {
        ICR3 = ocrValue;
    }
    else
    {
        ICR3 = 65535;
    }

    // Set the PWM mode to Fast PWM
    TCCR3A = (TCCR3A & 0b00111111) | 0b10100000;

    // Set the PWM pin as an output
    pinMode(pin, OUTPUT);

    // Set the PWM to 0 initially
    analogWrite(pin, 0);
}
