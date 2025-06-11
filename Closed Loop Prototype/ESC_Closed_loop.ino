#define PWM_MAX_DUTY      225
#define PWM_MIN_DUTY      25
#define PWM_START_DUTY    35
#define THROTTLE_PIN      8    
#define BEMF_FILTER_SAMPLES 4
volatile unsigned long PWM_INPUT = 0;
volatile unsigned long counter_1 = 0; 
volatile uint8_t last_PWM_state = 0;
volatile uint8_t bemfFilterCount = 0;  
volatile uint8_t bldc_step = 0;   
volatile uint8_t motor_speed = PWM_START_DUTY;// Mapped PWM duty cycle from throttle
volatile bool startup_done = false; // Flag to indicate transition from OL to CL

void setup() {
  // Configure low-side outputs (pins 3,4,5 on PORTD)
  DDRD |= 0x38;    // PD3, PD4, PD5 as outputs
  PORTD = 0x00;
  // Configure high-side outputs (pins 9,10,11 on PORTB)
  DDRB |= 0x0E;    // PB1, PB2, PB3 (digital 9, 10, 11) as outputs
  PORTB = 0x00;
  // Set up Timer1 and Timer2 for PWM (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  TCCR2A = 0;
  TCCR2B = 0x01;
  // Setup for Throttle Input Interrupt (PCINT) 
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  //  Setup for Analog Comparator (Back-EMF sensing) 
  ADCSRB |= (1 << ACME);
  ADCSRA &= ~(1 << ADEN);
  DIDR1 |= (1 << AIN0D) | (1 << AIN1D);
  ACSR =
    (0 << ACD)  |   // Analog Comparator enabled
    (0 << ACBG) |   // Use external voltage at AIN0+
    (0 << ACO)  |   // Clear output bit (read-only)
    (1 << ACI)  |   // Clear interrupt flag by writing a one
    (0 << ACIE) |   // Disable comparator interrupt 
    (0 << ACIC);    // Disable input capture
  startup_done = false;
  SET_PWM_DUTY(PWM_START_DUTY);
  sei();
}
void loop() {
  unsigned int pulseVal = PWM_INPUT;
  if (pulseVal < 1008) pulseVal = 1008;
  if (pulseVal > 2008) pulseVal = 2008;
  if (pulseVal == 1008) {
    SET_PWM_DUTY(0);           // Set PWM duty to 0 (motor off)
    PORTD = 0;                 // Clear low-side motor pins
    PORTB = 0;                 // Clear high-side motor pins
    ACSR &= ~(1 << ACIE);      // Disable comparator interrupt
    startup_done = false;      // Reset startup flag
  }
  else {
    // Map throttle input to PWM duty cycle
    motor_speed = (uint8_t) map(pulseVal, 1008, 2008, PWM_MIN_DUTY, PWM_MAX_DUTY);
    SET_PWM_DUTY(motor_speed);
    if (!startup_done) {
      openLoopStartup();  // Run open-loop startup if not completed
    }
  }
}
// Open Loop Startup Routine 
void openLoopStartup() {
  unsigned long delayTime = 4000;
  while (delayTime > 500) {
    if (PWM_INPUT <= 1008) {
      SET_PWM_DUTY(0);
      PORTD = 0;
      PORTB = 0;
      return;                  // Exit function early
    }
    delayMicroseconds(delayTime);
    bldc_move(bldc_step);      // Commutate motor
    bldc_step = (bldc_step + 1) % 6;  // Advance to next step
    delayTime -= 10;           // Gradually decrease delay
  }
  startup_done = true;         // Mark startup as complete
  ACSR |= (1 << ACIE);  // Enable comparator interrupt for CL
}
//  Analog Comparator ISR for Closed Loop Commutation 
ISR(ANALOG_COMP_vect) {
  static unsigned long lastCommutationTime = 0;
  unsigned long now = micros();
  const unsigned long minInterval = 100;// Minimum interval in µs to filter noise
  if (now - lastCommutationTime < minInterval) {
    return; // Too soon to commutate again
  }
  // Read the comparator output (ACO bit)
  bool currentState = (ACSR & (1 << ACO)) != 0;
  bool expectedState = (bldc_step % 2 == 0);
  if (currentState == expectedState) {
    bemfFilterCount++;
    if (bemfFilterCount >= BEMF_FILTER_SAMPLES) {
      bemfFilterCount = 0;
      lastCommutationTime = now;
      bldc_step = (bldc_step + 1) % 6;
      bldc_move(bldc_step);
    }
  } else {
    bemfFilterCount = 0;
  }
}
// Throttle Pulse Measurement ISR (PCINT0) 
ISR(PCINT0_vect) {
  unsigned long current_count = micros();
  // Check the state of the throttle pin (PB0)
  if (PINB & (1 << PINB0)) {
    if (last_PWM_state == 0) {  // Rising edge detected
      last_PWM_state = 1;
      counter_1 = current_count;
    }
  }
  else if (last_PWM_state == 1) {  // Falling edge detected
    last_PWM_state = 0;
    PWM_INPUT = current_count - counter_1;
  }
}
// Function: Set PWM Duty Cycle 
void SET_PWM_DUTY(uint8_t duty) {
  if (duty < PWM_MIN_DUTY)
    duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty = PWM_MAX_DUTY;
  OCR1A = duty;  // PWM duty for pin 9
  OCR1B = duty;  // PWM duty for pin 10
  OCR2A = duty;  // PWM duty for pin 11
}
// BLDC Commutation Functions 
void AH_BL() {
  PORTD = B00001000;  // Set PD3 HIGH
  TCCR2A = 0;         // Disable PWM on D11 (OC2A)
  TCCR1A = 0x81;      // PWM on D9 (OC1A) noninverting
}
void AH_CL() {
  PORTD = B00000100;  // Set PD2 HIGH
  TCCR2A = 0;
  TCCR1A = 0x81;
}
void BH_CL() {
  PORTD = B00000100;
  TCCR2A = 0;
  TCCR1A = 0x21;      // PWM on D10 (OC1B) noninverting
}
void BH_AL() {
  PORTD = B00010000;  // Set PD4 HIGH
  TCCR2A = 0;
  TCCR1A = 0x21;
}
void CH_AL() {
  PORTD = B00010000;  // Set PD4 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;      // PWM on D11 (OC2A) noninverting
}
void CH_BL() {
  PORTD = B00001000;  // Set PD3 HIGH
  TCCR1A = 0;
  TCCR2A = 0x81;
}
void bldc_move(uint8_t step) {
  switch (step) {
    case 0:
      AH_BL();
      BEMF_C_RISING();  // Expect rising edge on C-phase
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING(); // Expect falling edge on B-phase
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();  // Expect rising edge on A-phase
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING(); // Expect falling edge on C-phase
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();  // Expect rising edge on B-phase
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING(); // Expect falling edge on A-phase
      break;
  }
}
//  Back-EMF Comparator Configuration Functions 
void BEMF_A_RISING() {  
  ADCSRA &= ~(1 << ADEN);     // Disable ADC
  ADCSRB = (1 << ACME);   // Enable MUX for comparator negative input
  ADMUX = 2;            // Select A2 as comparator negative input
  ACSR |= 0x03;        // Configure for rising edge (ACIS1:0 = 11)
}
void BEMF_A_FALLING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 2;               // A2 for A-phase
  ACSR &= ~0x01;     // Configure for falling edge (ACIS1:0 = 10)
}
void BEMF_B_RISING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 1;            // Select A1 as comparator negative input
  ACSR |= 0x03          // Rising edge
}
void BEMF_B_FALLING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 1;                  // A1 for B-phase
  ACSR &= ~0x01;             // Falling edge
}
void BEMF_C_RISING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 0;             // Select A0 as comparator negative input
  ACSR |= 0x03;               // Rising edge
}
void BEMF_C_FALLING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 0;                  // A0 for C-phase
  ACSR &= ~0x01;             // Falling edge
}
