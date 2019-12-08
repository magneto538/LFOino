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

#define LED_PIN 11
#define LFO1_PIN 5
#define LFO2_PIN 6
#define RATE_PIN A0
#define WAVE_PIN A1
#define TAP_TEMPO_PIN 12

// Define Max and Min rate. May be floating point numbers
#define RATE_MIN 0.0
#define RATE_MAX 20.0

// Define Sample Rate. Used to handle phasor.
#define SR 31000.0

#define TAP_MAX 5

double cyclesNum = 0;
double cyclesNumPre = -1;

double ph = 0;
double rate;
double value;
double output;
double tap[TAP_MAX];
double delta;
double tapTime = 0;

int tapCount = 0;

int wave = 0;
int waveBtnState;
int tapBtnState;
int rateVal;
int waveVal;

unsigned long samplesUptime = 0;

void setup() {
  pinMode(LFO1_PIN, OUTPUT);
  pinMode(LFO2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RATE_PIN, INPUT);
  pinMode(WAVE_PIN, INPUT);
  pinMode(TAP_TEMPO_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  randomSeed(analogRead(0));

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
  rateVal = analogRead(RATE_PIN);

} 

// Timer0 interrupt operations. Used only to increment the phasor at the selected rate. This allows to avoid using delay()
// in the loop, which is far from precise.
extern "C"
{
  ISR(TIMER1_COMPA_vect){
    ph = ph + rate;

    if (ph >= SR) {
      ph = 0; 
      cyclesNum++;
    } 

    samplesUptime = (samplesUptime+1) % 0xFFFFFFFF;
    
  }
}

void loop() {
  rate = roundDecimalPlaces(potMap(RATE_PIN, RATE_MIN, RATE_MAX), 1);

  Serial.println(rate);
  wave = potMap(WAVE_PIN, 0, 8);

  if (digitalRead(TAP_TEMPO_PIN) == LOW && tapBtnState != digitalRead(TAP_TEMPO_PIN)) {
    if (tapCount == 0 || msUptime() - tapTime > 3000) {
      tapCount = 0;

      for(int i=0; i<TAP_MAX; i++){
        tap[i] = -1;
      } 

      tap[tapCount] = msUptime();
      ph = 0;
    }
    else {  
      if (tap[tapCount] != -1) {
        for(int i=0; i<TAP_MAX; i++){
          tap[i] = tap[i+1];
        } 
      }

      tap[tapCount-1] = msUptime() - tap[tapCount-1];
      tap[tapCount] = msUptime();

      delta = 0.0;
      for(int i=0; i<tapCount; i++){
        delta += tap[i];
      } 

      delta = (delta / tapCount) / 1000;
      rate = 1 / delta;
    }

    tapTime = msUptime();

    if (tapCount < TAP_MAX-1) {
      tapCount++;
    };
  };
  tapBtnState = digitalRead(TAP_TEMPO_PIN);  


  // Wave selection
  switch (wave) {
    case 0:
       value = waveSaw(ph, SR);
      break;

    case 1:
      value = waveSqr(ph, SR);
      break;

    case 2:
      value = waveTri(ph, SR);
      break;

    case 3:
      value = waveSH(ph, SR, cyclesNumPre != cyclesNum);
      break;

    case 4:
      value = waveSin(ph, SR, 1);
      break;

    case 5:
      value = waveSin(ph, SR, 2);
      break;

    case 6:
      value = waveSin(ph, SR, 3);
      break;

    case 7:
      value = waveSin(ph, SR, 4);
      break;

    case 8:
      value = waveSin(ph, SR, 5);
      break;

  }


  cyclesNumPre = cyclesNum;

  // Fire the current value to the outlets
  if (value != false) {
    output = ((value / 2.0) + 0.5) * 255;
    analogWrite(LED_PIN, output);
    analogWrite(LFO1_PIN, output);
    analogWrite(LFO2_PIN, output);
  }
} 

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double msUptime() {
  return (samplesUptime / SR) * 1000;
}

double potMap(int pin, double toLow, double toHigh) {
  return mapf(analogRead(pin), 0.0, 1023.0, toLow, toHigh);
}

double waveSaw(double ph, double sr) {
  return ((ph / sr) * 2) - 1;
}

double waveSin(double ph, double sr, int partials) {
  double output = 0;

  for (int i=0; i<partials; i++) {
    output = output + (i+1) * sin(((ph / sr) * 2*PI) / (i+1));
  }

  return output;
}

double waveSqr(double ph, double sr) {
  return (round(ph / sr) * 2) - 1;
}

double waveTri(double ph, double sr) {
  return (abs(waveSaw(ph, sr)) * 2) - 1;
}

double waveSH(double ph, double sr, double cond) {
  if (cond) {
    return random(-100, 100) / 100.0;
  }
  else {
    return false;
  }
}

double roundDecimalPlaces (double value, int n) {
  double p = pow(10, n);
  return round(value * p) / p;
}


