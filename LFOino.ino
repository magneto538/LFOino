#include <math.h> 
#define LED_PIN 6
#define POT_PIN A0
#define BUTTON_PIN 5
#define RATE_MIN 0.1
#define RATE_MAX 10.0

#define SR 32000.0

double ph = 0;
double brightness = 0;
double rate = 0;
double value = 0;

int wave = 0;
bool waveBtnState;



void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);
    Serial.begin(9600);
    cli();

    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    
    OCR1A = (16000000/SR)-1;
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // prescaler is 1
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    sei();
} 

extern "C"
{
  ISR(TIMER1_COMPA_vect){
    ph = ph + rate; 

    if (ph >= SR) {
      ph = 0;
    };
  }
}

void loop() {
    rate = map(analogRead(POT_PIN), 0, 1023, RATE_MIN, RATE_MAX);

    if (digitalRead(BUTTON_PIN) == 1 && waveBtnState != digitalRead(BUTTON_PIN)) {
      wave = (wave + 1) % 4;
    };
    waveBtnState = digitalRead(BUTTON_PIN);

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

    brightness = ((value / 2.0) + 0.5) * 255;
    analogWrite(LED_PIN, brightness);
} 
