/* LFOino - a CV LFO made with Arduino
by Davide Magni

Last Update - 2019 04 28

Resources:
https://playground.arduino.cc/Main/TimerPWMCheatsheet/
*/


#include <math.h> 

// Pinout: 
// D5 and D6: PWM outlets controlled by Timer0. Used for LFO and LED outputs
// D7: Shape selector inlet
// A0: Rate pot inlet

#define LED_PIN 6
#define SIGNAL_PIN 5
#define POT_PIN A0
#define BUTTON_PIN 7
#define TAP_TEMPO_PIN 8

// Define Max and Min rate. May be floating point numbers
#define RATE_MIN 0.01
#define RATE_MAX 20.0

// Define Sample Rate. Used to handle phasor.
#define SR 32000

double ph;
double rate;
double value;
double tap[4];
int tapCount;

int wave = 0;
int waveBtnState;
int tapBtnState;
int potVal;

void setup() {
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  cli(); // stop interrupts

  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0

  // Timer0
  // Timer0 is forced at 62kHz and governs PWM. The frequency is that high in order to avoid aliasing
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;

  // Timer1
  // Timer1 is used to handle only the phasor increment. Triggers an interrupt at regular intervals defined by the sample rate.
  // Allows for better time-sync operations
  OCR1A = (16000000 / SR) - 1;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); // allow interrupts
  
  // Additional config things
  potVal = analogRead(POT_PIN);

} 

// Timer0 interrupt operations. Used only to increment the phasor at the selected rate. This allows to avoid using delay()
// in the loop, which is far from precise.
extern "C"
{
  ISR(TIMER1_COMPA_vect){
    ph = ph + rate;

    if (ph >= SR) {
      ph = 0;
    }
  }
}

void loop() {
  // Scale the pot value into a double-type value. Mapf allows to map doubles instead of int.
  if (potVal != analogRead(POT_PIN)) {
    rate = mapf(analogRead(POT_PIN), 0, 1023, RATE_MIN, RATE_MAX);
  }
  potVal = analogRead(POT_PIN);

  // Button handling. Choose the next waveform
  if (digitalRead(BUTTON_PIN) == HIGH && waveBtnState != HIGH) {
    wave = (wave + 1) % 4;
  };
  waveBtnState = digitalRead(BUTTON_PIN);

  if (digitalRead(TAP_TEMPO_PIN) == HIGH && tapBtnState != HIGH) {
    if (tapCount > 0) {
      tap[tapCount] = rate - tap[tapCount-1];

      for(int i=0; i<tapCount; i++){
        rate = (tap[i]) / tapCount;
      }
    }
    else {
      tap[tapCount] = rate;
    }

    tapCount = (tapCount+1) % 4;
  };
  tapBtnState = digitalRead(TAP_TEMPO_PIN);  

  // Wave selection
  switch (wave) {
    case 0:
       value = ((ph / SR) * 2) - 1;
      break;

    case 1:
      value = sin((ph / SR) * 2*PI);
      break;

    case 2:
      if (ph < SR/2) {
        value = 1.0;
      }
      else {
        value = -1.0;
      };
      break;

    case 3:
      value = ph - SR/2;
      value = (0.002 * abs(value) * 2) - 1;
      break;
  }

  // Fire the current value to the outlets
  value = ((value / 2.0) + 0.5) * 255;
  analogWrite(LED_PIN, value);
  analogWrite(SIGNAL_PIN, value);
} 

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

