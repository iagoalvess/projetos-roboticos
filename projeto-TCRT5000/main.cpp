#include <Arduino.h>
// LED infravermelho no D8, coletor do fototransistor no A0.
// Comecem com 1 s ligado e 1 s desligado. Depois reduzam os dois.
const int PIN_LED = 8;
const int PIN_ADC = A0;
unsigned long t_on_ms = 1000;
unsigned long t_off_ms = 1000;

const float G0 = -102.2058;
const float G1 = 1845.4711;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(PIN_LED, HIGH);
  delay(t_on_ms);
  int adc_on = analogRead(PIN_ADC);
  digitalWrite(PIN_LED, LOW);
  delay(t_off_ms);
  int adc_off = analogRead(PIN_ADC);
  int sinal = adc_off - adc_on;

  if (sinal > 0) {
    float distancia = G0 + G1 / sqrt((float)sinal);

    Serial.print(sinal);
    Serial.print(",");
    Serial.println(distancia);
  } else {
    Serial.print(sinal);
    Serial.println(",-1");
  }

}
