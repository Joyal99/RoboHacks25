#define RIGHTM_FOWARD_PIN PD4
#define RIGHTM_BACK_PIN PD5
#define LEFTM_FOWARD_PIN PD6
#define LEFTM_BACK_PIN PD7

void motorPinSetup(){
  // Set motor pins as outputs
  DDRD |= (1<<RIGHTM_FOWARD_PIN) | (1<<LEFTM_FOWARD_PIN) | (1<<RIGHTM_BACK_PIN) | (1<<LEFTM_BACK_PIN);
}

void turnLeft(){
  // Pivot left: Right motor forward, Left motor backward
  PORTD |= (1<<RIGHTM_FOWARD_PIN);
  PORTD &= ~(1<<RIGHTM_BACK_PIN);
  PORTD &= ~(1<<LEFTM_FOWARD_PIN);
  PORTD |= (1<<LEFTM_BACK_PIN);
  _delay_ms(10); // Tiny delay before stopping
  stop();
}

void turnRight(){
  // Pivot right: Left motor forward, Right motor backward
  PORTD |= (1<<LEFTM_FOWARD_PIN);
  PORTD &= ~(1<<LEFTM_BACK_PIN);
  PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
  PORTD |= (1<<RIGHTM_BACK_PIN);
  _delay_ms(10); // Tiny delay before stopping
  stop();
}

void moveFoward(){
  // Both motors forward
  PORTD &= ~(1<<LEFTM_BACK_PIN);
  PORTD &= ~(1<<RIGHTM_BACK_PIN);
  PORTD |= (1<<LEFTM_FOWARD_PIN);
  PORTD |= (1<<RIGHTM_FOWARD_PIN);
}

void moveBackward(){
  // Both motors backward
  PORTD &= ~(1<<LEFTM_FOWARD_PIN);
  PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
  PORTD |= (1<<LEFTM_BACK_PIN);
  PORTD |= (1<<RIGHTM_BACK_PIN);
}

void stop(){
  // Stop both motors immediately
  PORTD &= ~(1<<LEFTM_BACK_PIN);
  PORTD &= ~(1<<RIGHTM_BACK_PIN);
  PORTD &= ~(1<<LEFTM_FOWARD_PIN);
  PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
}
