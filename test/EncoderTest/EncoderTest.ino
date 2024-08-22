#include "Encoder.h"

////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////

// TOGLGLE MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the engaged direction
#define PIN_TOGGLE_DIR 12   
#define PIN_TOGGLE_PWM 3
#define PIN_TOGGLE_CURRENT A0     
   
// LINEAR MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the down shift direction
#define PIN_LINEAR_PWM 11
#define PIN_LINEAR_DIR 13     
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


#define NUM_MOTORS 2

enum Motors {
  TOGGLE=0,
  LINEAR=1,
};

// ENCODERS
Encoder encoders[NUM_MOTORS] = {
  Encoder(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B),
  Encoder(PIN_LINEAR_ENC_A, PIN_LINEAR_ENC_B) //14PPR Of the motor and gear ratio of 1:50 and 1/2 rev per index, but empiracally found 767 counts per index
};

void setup(){
  Serial.begin(115200);
}

void loop(){
  long encoderValue = encoders[TOGGLE].read();
  Serial.println(encoderValue);
  delay(500);
}