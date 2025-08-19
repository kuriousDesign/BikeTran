#include <TimerOne.h>
/////////////////////////////////////////////////////
//CONFIGURATION
/////////////////////////////////////////////////////

#define MOTOR_TO_TEST 0 // JOE, change this value to 0 to run toggle motor, 1 to run linear motor

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
#define PIN_EINK_BIT0 22      // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER
#define NUM_BITS 4
#define PIN_LINEAR_POS_LIM 41 
#define PIN_TOGGLE_NEG_LIM 43          
#define PIN_SHIFT_UP 51       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 53     // RED WIRE ON SHIFTER

void setup(){
    pinMode(PIN_TOGGLE_PWM, OUTPUT);
    pinMode(PIN_TOGGLE_DIR, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_CURRENT, INPUT);
    Serial.begin(115200);
    Serial.println("Motor Spin Test Script, hold up or down shift to spin motor");
      // Initialize Timer1
    Timer1.initialize(500000); // Set a period in microseconds (e.g., 1000 microseconds = 1 millisecond)
    Timer1.attachInterrupt(timerISR); // Attach the timer interrupt service routine (ISR)
}

// Timer interrupt service routine (ISR)
void timerISR() {
  //Serial.print("Analog output: ");
  if(MOTOR_TO_TEST == 0){
    //Serial.println(analogRead(PIN_TOGGLE_CURRENT));
  } else {
    //Serial.println(analogRead(PIN_LINEAR_CURRENT));
  }
}

void loop(){
    if(MOTOR_TO_TEST == 0){
        runToggleMotor();
    } else {
        runLinearMotor();
    }
    delay(1);
}

void runToggleMotor(){
    if(digitalRead(PIN_SHIFT_UP) == LOW){
        analogWrite(PIN_TOGGLE_PWM, 255);
        digitalWrite(PIN_TOGGLE_DIR, LOW);
        Serial.println(analogRead(PIN_TOGGLE_CURRENT));
    } else if(digitalRead(PIN_SHIFT_DOWN) == LOW){
        analogWrite(PIN_TOGGLE_PWM, 255);
        digitalWrite(PIN_TOGGLE_DIR, HIGH);
        //Serial.println(analogRead(PIN_TOGGLE_CURRENT));
    } else {
        digitalWrite(PIN_TOGGLE_PWM, LOW);
    }
}

void runLinearMotor(){
    if(digitalRead(PIN_SHIFT_UP) == LOW){
        analogWrite(PIN_LINEAR_A_PWM, 200);
        digitalWrite(PIN_LINEAR_DIR, HIGH);
    } else if(digitalRead(PIN_SHIFT_DOWN) == LOW){
        analogWrite(PIN_LINEAR_A_PWM, 200);
        digitalWrite(PIN_LINEAR_DIR, LOW);
    } else {
        digitalWrite(PIN_LINEAR_A_PWM, LOW);
    }
}