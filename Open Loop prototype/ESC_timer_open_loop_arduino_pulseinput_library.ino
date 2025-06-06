#include <PulseInput.h>
#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      50
#define PWM_START_DUTY    100
#define THROTTLE_PIN      8    // Transmitter throttle input (PD8)
volatile unsigned int rxPulse; 
byte bldc_step = 0, motor_speed;
unsigned int i;
void setup() {
  DDRD  |= 0x38;   // PD3, PD4, PD5 as outputs
  PORTD  = 0x00;
  DDRB  |= 0x0E;   // PB1, PB2, PB3 (digital 9,10,11) as outputs
  PORTB  = 0x00;
  // Timer1 and Timer2 setup for PWM (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  TCCR2A = 0;
  TCCR2B = 0x01;
  attachPulseInput(THROTTLE_PIN, rxPulse);
  SET_PWM_DUTY(PWM_START_DUTY);
}
void loop() {
  unsigned int pulseVal = rxPulse;
  if (pulseVal < 1008) pulseVal = 1008;
  if (pulseVal > 2008) pulseVal = 2008;
 unsigned int v_delay = map(pulseVal, 1008, 2008, 4000, 850);  
  motor_speed = map(pulseVal, 1008, 2008, PWM_MIN_DUTY, PWM_MAX_DUTY);
  SET_PWM_DUTY(motor_speed);
  delayMicroseconds(v_delay);
  bldc_move();
  bldc_step = (bldc_step + 1) % 6;
}
// BLDC commutation function (open-loop)
void bldc_move() {
  switch(bldc_step) {
    case 0:
      AH_BL();
      break;
    case 1:
      AH_CL();
      break;
    case 2:
      BH_CL();
      break;
    case 3:
      BH_AL();
      break;
    case 4:
      CH_AL();
      break;
    case 5:
      CH_BL();
      break;
  }
}
void SET_PWM_DUTY(byte duty) {
  if (duty < PWM_MIN_DUTY)
    duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty = PWM_MAX_DUTY;
  OCR1A = duty;  // PWM duty for pin 9
  OCR1B = duty;  // PWM duty for pin 10
  OCR2A = duty;  // PWM duty for pin 11
}
// Commutation functions for BLDC motor phases
void AH_BL(){
  PORTD = B00001000;      // Set D3 HIGH
  TCCR2A = 0;             // D11 normal port
  TCCR1A = 0x81;          // PWM on D9 (OC1A) in noninverting mode
}
void AH_CL(){
  PORTD = B00000100;      // Set D2 HIGH
  TCCR2A = 0;
  TCCR1A = 0x81;
}
void BH_CL(){
  PORTD = B00000100;      
  TCCR2A = 0;
  TCCR1A = 0x21;          // PWM on D10 (OC1B)
}
void BH_AL(){  
  PORTD = B00010000;      // Set D4 HIGH
  TCCR2A = 0;
  TCCR1A = 0x21;
}
void CH_AL(){ 
  PORTD = B00010000;      // Set D4 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;          // PWM on D11 (OC2A)
}
void CH_BL(){  
  PORTD = B00001000;      // Set D3 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;
}
