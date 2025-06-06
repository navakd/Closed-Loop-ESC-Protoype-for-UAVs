#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      25
#define PWM_START_DUTY    35
#define THROTTLE_PIN      8    
volatile unsigned long PWM_INPUT = 0;  // Latest measured pulse width from throttle (in µs)
volatile unsigned long counter_1 = 0;// Timestamp at the rising edge
volatile byte last_PWM_state = 0;   // Last known state of the throttle pin
byte bldc_step = 0;       // Current commutation step (0-5)
byte motor_speed = 0;  // Mapped PWM duty cycle based on throttle input
unsigned long last_commutation = 0;// Time when the last commutation step was executed
void setup() {
  // Configure low-side outputs (pins 3,4,5 on PORTD)
  DDRD |= 0x38;    // Set PD3, PD4, PD5 as outputs
  PORTD = 0x00;
  // Configure high-side outputs (pins 9,10,11 on PORTB)
  DDRB |= 0x0E;    // Set PB1, PB2, PB3 (digital 9, 10, 11) as outputs
  PORTB = 0x00;
  // Set up Timer1 and Timer2 for PWM (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  TCCR2A = 0;
  TCCR2B = 0x01;
  // Configure Pin Change Interrupt on THROTTLE_PIN 8
  PCICR |= (1 << PCIE0);
  // Enable interrupt specifically for PCINT0 (pin 8 corresponds to PCINT0 on PORTB)
  PCMSK0 |= (1 << PCINT0);
  sei();
  // Set an initial PWM duty cycle
  SET_PWM_DUTY(PWM_START_DUTY);
}
void loop() { 
  unsigned int pulseVal = PWM_INPUT;
  if (pulseVal < 1008) pulseVal = 1008;
  if (pulseVal > 2008) pulseVal = 2008;
  unsigned int v_delay = map(pulseVal, 1008, 2008, 4000, 850);
  // Map the pulse width to a PWM duty cycle (for example, from PWM_MIN_DUTY to PWM_MAX_DUTY)
  motor_speed = map(pulseVal, 1008, 2008, PWM_MIN_DUTY, PWM_MAX_DUTY);
  SET_PWM_DUTY(motor_speed);
  if (micros() - last_commutation >= v_delay) {
    bldc_move();       // Execute the current commutation step
    bldc_step = (bldc_step + 1) % 6; // Cycle through the 6-step commutation
    last_commutation = micros();      // Reset the timer for the next step
  }
}
// This ISR measures the pulse width from the throttle transmitter.
// It triggers on any change on PCINT0 (which includes pin 8).
ISR(PCINT0_vect) {
  unsigned long current_count = micros(); 
  if (PINB & B00000001) {
    if (last_PWM_state == 0) {   // Rising edge detected
      last_PWM_state = 1;
      counter_1 = current_count; // Capture the time at the rising edge
    }
  }
  else if (last_PWM_state == 1) {  // Falling edge detected
    last_PWM_state = 0;
    PWM_INPUT = current_count - counter_1;  // Calculate the pulse width
  }
}
// Set PWM Duty Cycle 
void SET_PWM_DUTY(byte duty) {
  if (duty < PWM_MIN_DUTY)
    duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty = PWM_MAX_DUTY;
  OCR1A = duty;  // PWM duty for pin 9
  OCR1B = duty;  // PWM duty for pin 10
  OCR2A = duty;  // PWM duty for pin 11
}
//BLDC Commutation Functions 
void bldc_move() {
  switch (bldc_step) {
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
void AH_BL() {
  PORTD = B00001000;  // Set PD3 HIGH
  TCCR2A = 0;         // Ensure D11 (OC2A) is in normal (non-PWM) mode
  TCCR1A = 0x81;      // PWM on D9 (OC1A) in noninverting mode
}
void AH_CL() {
  PORTD = B00000100;  // Set PD2 HIGH
  TCCR2A = 0;
  TCCR1A = 0x81;
}
void BH_CL() {
  PORTD = B00000100;
  TCCR2A = 0;
  TCCR1A = 0x21;      // PWM on D10 (OC1B) in noninverting mode
}
void BH_AL() {
  PORTD = B00010000;  // Set PD4 HIGH
  TCCR2A = 0;
  TCCR1A = 0x21;
}
void CH_AL() {
  PORTD = B00010000;  // Set PD4 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;      // PWM on D11 (OC2A) in noninverting mode
}
void CH_BL() {
  PORTD = B00001000;  // Set PD3 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;
}
