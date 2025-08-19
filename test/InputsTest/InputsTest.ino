////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////

// TOGLGLE MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the engaged direction
#define PIN_TOGGLE_DIR 4   
#define PIN_TOGGLE_PWM 5
#define PIN_TOGGLE_CURRENT A0     
   
// LINEAR MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the down shift direction
#define PIN_LINEAR_PWM 6
#define PIN_LINEAR_DIR 7     
#define PIN_LINEAR_CURRENT A1    
  
#define PIN_LINEAR_ENC_A 18   //
#define PIN_LINEAR_ENC_B 19   //
#define PIN_TOGGLE_ENC_A 20   //
#define PIN_TOGGLE_ENC_B 21   //
#define PIN_EINK_BIT0 22      // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER DISPLAY ON E-INK
#define NUM_BITS 4
#define PIN_LINEAR_POS_LIM 41 
#define PIN_TOGGLE_NEG_LIM 43          
#define PIN_SHIFT_UP 51       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 53     // RED WIRE ON SHIFTER

struct Inputs
{
  bool ShiftDownSw = false;
  bool ShiftUpSw = false;
  bool ClutchNegLimSw = false;
  bool LinearPosLimSw = false;
  uint16_t ToggleCurrent = 0;
  uint16_t LinearCurrent = 0;
};
Inputs inputs;
Inputs lastInputs;

bool firstScan = true;
void readInputs()
{
  inputs.ShiftUpSw = !digitalRead(PIN_SHIFT_UP);
  if (inputs.ShiftUpSw != lastInputs.ShiftUpSw || firstScan)
  {
    Serial.print("Shift Up Switch: ");
    Serial.println(inputs.ShiftUpSw);
  }
  inputs.ShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
  if (inputs.ShiftDownSw != lastInputs.ShiftDownSw || firstScan)
  {
    Serial.print("Shift Down Switch: ");
    Serial.println(inputs.ShiftDownSw);
  }
  inputs.ClutchNegLimSw = !digitalRead(PIN_TOGGLE_NEG_LIM);
    if (inputs.ClutchNegLimSw != lastInputs.ClutchNegLimSw || firstScan)
    {
        Serial.print("Toggle Negative Limit Switch: ");
        Serial.println(inputs.ClutchNegLimSw);
    }
  inputs.LinearPosLimSw = !digitalRead(PIN_LINEAR_POS_LIM);
    if (inputs.LinearPosLimSw != lastInputs.LinearPosLimSw || firstScan)
    {
        Serial.print("Linear Positive Limit Switch: ");
        Serial.println(inputs.LinearPosLimSw);
    }
    firstScan = false;
}

void setup()
{
  pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
  pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_NEG_LIM, INPUT_PULLUP);
  pinMode(PIN_LINEAR_POS_LIM, INPUT_PULLUP);
  Serial.begin(115200);

}

void loop(){
    readInputs();
    lastInputs = inputs;
}