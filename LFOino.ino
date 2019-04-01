#include <math.h> 
#define LED_PIN 3
#define POT_PIN A0
#define BUTTON_PIN A1
#define RATE_MIN 1.0
#define RATE_MAX 10.0

double ph = 0;
double brightness = 0;
double rate = 0;
double value = 0;

int wave = 0;
int waveBtnState;

double time = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
	rate = map(analogRead(POT_PIN), 0, 1023, RATE_MIN, RATE_MAX);
	ph = ph + rate; 

	if (ph >= 1000) {
		ph = 0;
	};

	if (analogRead(BUTTON_PIN) != waveBtnState && analogRead(BUTTON_PIN) == 1023) {
		wave = (wave+1) % 4;
	};
	waveBtnState = analogRead(BUTTON_PIN);

    switch (wave) {
      case 0:
		value = ((ph / 1000.0) * 2) - 1;
        break;

      case 1:
		value = sin((ph / 1000.0) * 2*PI);
        break;

      case 2:
      	if (ph < 500) {
      		value = 1.0;
      	}
      	else {
      		value = -1.0;
      	};
        break;

      case 3:
      	value = ph - 500;
      	value = (0.002 * abs(value) * 2) - 1;
        break;
    }

	brightness = ((value / 2.0) + 0.5) * 255;
	analogWrite(LED_PIN, brightness);

	delayMicroseconds(1000);
} 