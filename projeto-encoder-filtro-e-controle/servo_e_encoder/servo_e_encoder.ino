#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

int ENCODER_A = 2;     // canal A do encoder (pino com interrupção)
int ENCODER_B = 3;     // canal B do encoder (pino com interrupção)
int SERVO_MOT_PIN = 9; // sinal do servo

// A biblioteca configura os pinos, liga as interrupções e conta os pulsos
// sozinha. enc.read() devolve a posição acumulada (em contagens).
Encoder enc(ENCODER_A, ENCODER_B);

Servo m1;

// Largura de pulso mínima e máxima aceitas pelo servo (microssegundos).
const int M1_US_MIN = 544;
const int M1_US_MAX = 2500;

// Servo de rotação contínua: a largura do pulso define velocidade e sentido,
int max_sentido_horario = 2000;
int max_sentido_antihorario = 1000;
int parado = 1500;

int giro_lento_antihorario = 1400;
int giro_lento_horario = 1600;

// ---------------------------------------------------------------------------
// setup(): roda uma vez quando a placa liga
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600); // mesma velocidade do monitor_speed / Teleplot

  m1.attach(SERVO_MOT_PIN, M1_US_MIN, M1_US_MAX);
  delay(2000);

  // Defineelocidade
  m1.writeMicroseconds(giro_lento_horario);
}

// ---------------------------------------------------------------------------
// Filtro passa-baixa de primeira ordem
//   y[k] = y[k-1] + alfa * (x[k] - y[k-1])
// alfa perto de 0 -> filtra muito (resposta lenta);
// alfa perto de 1 -> filtra pouco (segue o sinal bruto).
// ---------------------------------------------------------------------------
float y = 0; // saída do filtro
float passaBaixa(float x, float alfa) { y += alfa * (x - y); return y; }


// ---------------------------------------------------------------------------
// Controlador PI
//   r = referência (velocidade desejada), y = velocidade medida
//   u = Kp * e + Ki * soma(e)
// ---------------------------------------------------------------------------
float u = 0;   // saída do controle
float e = 0;   // erro atual
float e_i = 0; // erro acumulado (parte integral)

float controlePID(float r, float y) {
  e = y - r;       // erro (atenção: o usual é r - y)
  e_i = e_i + e;   // integra o erro
  float Kp = 2;    // ganho proporcional
  float Ki = 0.1;  // ganho integral

  u = Kp * e + Ki * e_i;
  return u;
}

// k = pi * D / N
// D = 4.4
// N = 288
// k = 0.47997
float k = 0.47997;   // mm por contagem (pi * D / N)

long prevpos = -999;     // posição do encoder na última impressão
unsigned long timer = 0; // instante do último ciclo (para o período de 50 ms)

long ti = 0; // instante da medição anterior (ms)
long tf = 0; // instante da medição atual (ms)

void loop() {
  long pos = enc.read(); // posição acumulada do encoder (contagens)
  tf = millis();         // instante desta leitura

  // Comandos pela serial (ex.: campo de envio do Teleplot):
  //   's' -> para a roda    'g' -> volta a girar
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') m1.writeMicroseconds(parado);
    if (c == 'g') m1.writeMicroseconds(giro_lento_antihorario);
  }

  // Espera ocupada até completar 50 ms desde o ciclo anterior.
  // Garante uma taxa de amostragem fixa de ~20 Hz.
  while (millis() - timer < 50)
    ;
  timer = millis();

  // Só calcula e imprime se a roda se moveu desde a última vez.
  if (prevpos != pos) {
    // Contagens -> cm: (circunferência da roda = 4.4*pi cm) / (contagens por volta = 350)
    float distancia = (pos - prevpos) * (4.4 * 3.1415 / 350); // cm

    float tempo = tf - ti; // tempo entre as duas leituras, em ms
    tempo /= 1000.0;       // ms -> s

    float v = distancia/tempo;      // velocidade bruta (cm/s), bem ruidosa
    float vf = passaBaixa(v, 0.05); // velocidade filtrada (cm/s)

    // formato Teleplot: ">nome:valor" (um valor por linha)
    Serial.print(">pos:");
    Serial.println(pos);
    Serial.print(">distancia:");
    Serial.println(distancia);
    Serial.print(">tempo:");
    Serial.println(tempo);
    Serial.print(">v:");
    Serial.println(v);
    Serial.print(">vf:");
    Serial.println(vf);

    // Guarda a leitura atual como "anterior" para o próximo ciclo.
    prevpos = pos;
    ti = tf;
  }
}


// ---------------------------------------------------------------------------
// Contas de referência (velocidade máxima do servo e resolução do encoder)
// ---------------------------------------------------------------------------

// w = 750 graus por segundo
// w = 13.09 rad/s
// Diametro = 4.4 -> raio 2.2
// v = w r = 750/180 pi * 4.4 = 28.8 cm / s
// C = 2 pi r = 4.4pi = 13.82 cm
// tempo de 1 volta = 0.24 s

