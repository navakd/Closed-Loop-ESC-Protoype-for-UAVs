// --- Definitions and Constants ---
#define PWM_MAX_DUTY      180
#define PWM_MIN_DUTY      25
#define PWM_START_DUTY    35
#define THROTTLE_PIN      8    // Throttle transmitter input 

// --- Global Variables ---
volatile unsigned long PWM_INPUT = 0;  // Latest measured pulse width from throttle (in µs)
volatile unsigned long counter_1 = 0;    // Timestamp at the rising edge
volatile uint8_t last_PWM_state = 0;       // Last known state of the throttle pin

volatile uint8_t bemfFilterCount = 0;      // Filter counter for comparator noise rejection

// Commutation state variables
volatile uint8_t bldc_step = 0;    // Current commutation step (0-5)
volatile uint8_t motor_speed = PWM_START_DUTY;  // Mapped PWM duty cycle from throttle
volatile bool startup_done = false;  // Flag to indicate transition from open-loop to closed-loop

// Global flag to indicate if motor is enabled (true) or shut down (false)
volatile bool motor_enabled = true;

// New variables for adaptive filtering
volatile uint8_t adaptiveFilterThreshold = 8; // Starting value
//volatile uint8_t bemfFilterCount = 0;
volatile uint16_t commutationPeriod = 0;      // Time between commutations in microseconds
volatile unsigned long lastCommutationTime = 0;
volatile uint8_t commutationHistory[8] = {0}; // Store recent commutation periods
volatile uint8_t commutationHistoryIndex = 0;
volatile uint16_t averageCommutationPeriod = 500; 
void setup() {
  // Configure low-side outputs (pins 3,4,5 on PORTD)
  DDRD |= 0x1C;    // PD2, PD3, PD4 as outputs
  PORTD = 0x00;
  // Configure high-side outputs (pins 9,10,11 on PORTB)
  DDRB |= 0x0E;    // PB1, PB2, PB3 (digital 9, 10, 11) as outputs
  PORTB = 0x00;
  // Set up Timer1 and Timer2 for PWM (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;   // Clock enabled (no prescaling)
  TCCR2A = 0;
  TCCR2B = 0x01;   // Clock enabled (no prescaling)
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  //Setup for Analog Comparator (Back-EMF sensing) 
  // Enable MUX for negative input of comparator
  ADCSRB |= (1 << ACME);
  ADCSRA &= ~(1 << ADEN);
  DIDR1 |= (1 << AIN0D) | (1 << AIN1D);  
  ACSR =
    (0 << ACD)  |   // Analog Comparator enabled (ACD = 0)
    (0 << ACBG) |   // Use external voltage at AIN0+
    (0 << ACO)  |   // Clear output bit (read-only)
    (1 << ACI)  |   // Clear interrupt flag by writing a one
    (0 << ACIE) |   // Disable comparator interrupt for now (open-loop startup)
    (0 << ACIC);    // Disable input capture
  startup_done = false;
  motor_enabled = true;
  SET_PWM_DUTY(PWM_START_DUTY);
  sei();
}
void loop() {
  // Read the latest measured PWM input value (in µs) from the ISR
  unsigned int pulseVal = PWM_INPUT;
  // Clamp pulseVal to the expected range (e.g., 1008 µs to 2008 µs)
  if (pulseVal < 1008) pulseVal = 1008;
  if (pulseVal > 2008) pulseVal = 2008;
  if (pulseVal == 1008) {
    motor_enabled = false;
    shutdownMotor();
  }
  else {
    if (!motor_enabled) {
      // Reinitialize timers, comparator, and output pins when recovering from shutdown.
      reinitializeMotorOutputs();
      motor_enabled = true;
      startup_done = false;  // Force an open-loop restart
    }
    motor_speed = (uint8_t) map(pulseVal, 1008, 2008, PWM_MIN_DUTY, PWM_MAX_DUTY);
    SET_PWM_DUTY(motor_speed);
    // If not already in closed-loop mode, run the open loop startup routine
    if (!startup_done) {
      openLoopStartup();
    }
  }
}
void reinitializeMotorOutputs() {
  // Re-enable PWM timers
  TCCR1B = 0x01;   // Restart Timer1 clock
  TCCR2B = 0x01;   // Restart Timer2 clock
  ACSR &= ~(1 << ACD);
  DDRD |= 0x1C;    // PD2, PD3, PD4 as outputs
  DDRB |= 0x0E;    // PB1, PB2, PB3 as outputs
}
// Shutdown Motor Function 
void shutdownMotor() {
  // Disable comparator interrupt and disable comparator by setting ACD.
  ACSR |= (1 << ACD);  // Disable analog comparator
  ACSR &= ~(1 << ACIE); // Disable comparator interrupt
  TCCR1B = 0;
  TCCR2B = 0;
  // Set PWM duty to 0
  SET_PWM_DUTY(0);
  // Clear output ports so no phase is driven
  PORTD = 0;
  PORTB = 0;
}
// Open Loop Startup Routine 
void openLoopStartup() {
  // Before starting, ensure timers and comparator are enabled.
  TCCR1B = 0x01;
  TCCR2B = 0x01;
  ACSR &= ~(1 << ACD);  // Ensure comparator is enabled
  unsigned long delayTime = 1500;  // Start with a long delay (µs)
  while (delayTime > 500) { // Transition to closed loop: enable comparator interrupt
    if (!motor_enabled) {  // Abort startup if motor is disabled
      shutdownMotor();
      return;
    }
    delayMicroseconds(delayTime);
    // Advance the commutation step (open-loop)
    bldc_move(bldc_step);
    bldc_step = (bldc_step + 1) % 6;
    delayTime -= 10;
  }
  startup_done = true;
  ACSR |= (1 << ACIE);// Transition to closed loop: enable comparator interrupt
}
void updateMotorMetrics() {
  const uint8_t POLE_PAIRS = 7;
  if (averageCommutationPeriod > 0) {
    // Calculate electrical RPM
    uint32_t electricalRPM = 60000000UL / (averageCommutationPeriod * 7);
    // Calculate mechanical RPM
    uint32_t mechanicalRPM = electricalRPM / POLE_PAIRS;
    // Serial.print("RPM: ");    // Optional for Serial for debugging
    // Serial.println(mechanicalRPM);
  }
}
// Analog Comparator ISR for Closed Loop Commutation
ISR(ANALOG_COMP_vect) {
  unsigned long now = micros();
  unsigned long timeSinceLastCommutation = now - lastCommutationTime;
  // Adaptive minimum interval protection - shorter at higher speeds
  uint16_t minInterval = constrain(averageCommutationPeriod / 450, 380, 5000);
  if (timeSinceLastCommutation < minInterval) {
    return;
  }
  bool currentState = (ACSR & (1 << ACO));
  bool expectedState;
  // Even steps (0,2,4) expect rising edge (HIGH)
  // Odd steps (1,3,5) expect falling edge (LOW)
  if (bldc_step % 2 == 0) {
    expectedState = true; // Rising edge, comparator = HIGH
  } else {
    expectedState = false; // Falling edge, comparator = LOW
  }
  if (currentState == expectedState) {
    bemfFilterCount++;
    // Calculate adaptive threshold based on motor speed 
    adaptiveFilterThreshold = constrain(1 - (motor_speed / 3), 3 , 6);
    
    if (bemfFilterCount >= adaptiveFilterThreshold) {
      // Valid zero-crossing detected - time to commutate
      // Record commutation period for adaptive timing
      if (lastCommutationTime > 0) {
        commutationPeriod = now - lastCommutationTime;
        
        // Store in history array
        commutationHistory[commutationHistoryIndex] = commutationPeriod;
        commutationHistoryIndex = (commutationHistoryIndex + 1) % 8;
        uint32_t sum = 0; // Calculate average of last 8 commutation periods
        for (uint8_t i = 0; i < 8; i++) {
          sum += commutationHistory[i];
        }
        averageCommutationPeriod = sum / 8;
      } 
      bemfFilterCount = 0;  // Reset for next detection
      lastCommutationTime = now; 
      bldc_step = (bldc_step + 1) % 6; 
      bldc_move(bldc_step);
    }
  } else {        // Wrong state detected, reset counter
    bemfFilterCount = 0;
  }
}
// Throttle Pulse Measurement ISR (PCINT0) .
ISR(PCINT0_vect) {
  unsigned long current_count = micros();
  // Check the state of the throttle pin (PB0 holds the state for PCINT0)
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
void SET_PWM_DUTY(uint8_t duty) {
  if (duty < PWM_MIN_DUTY)
    duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty = PWM_MAX_DUTY;
  OCR1A = duty;  // PWM duty for pin 9
  OCR1B = duty;  // PWM duty for pin 10
  OCR2A = duty;  // PWM duty for pin 11
}
void AH_BL() {
  PORTD = B00001000;  // Set PD3 HIGH
  TCCR2A = 0;         // Disable PWM on D11 (OC2A) - normal mode
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
  if (!motor_enabled) return;
  switch (step) {
    case 0:
      AH_BL();
      BEMF_C_RISING();// Expect rising edge on C-phase zero-crossing
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
void BEMF_A_RISING() {  
  ADCSRA &= ~(1 << ADEN);     // Disable ADC
  ADCSRB = (1 << ACME);  // Enable MUX for comparator negative input
  ADMUX = 2;            // Select A2 as comparator negative input
  ACSR |= 0x03;            // Configure for rising edge 
}
void BEMF_A_FALLING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 2;                // A2 for A-phase
  ACSR &= ~0x01;           // Configure for falling edge 
}
void BEMF_B_RISING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 1;               // Select A1 as comparator negative input
  ACSR |= 0x03;              // Rising edge
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
  ACSR |= 0x03;              // Rising edge
}
void BEMF_C_FALLING() {
  ADCSRA &= ~(1 << ADEN);
  ADCSRB = (1 << ACME);
  ADMUX = 0;                  // A0 for C-phase
  ACSR &= ~0x01;             // Falling edge
}
