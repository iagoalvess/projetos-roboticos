#include<Servo.h>

//#include "C:\utility\direct_pin_read.h"
#include "C:\Users\renan.g.oliveira\OneDrive - Accenture\Documents\Arduino\servo_e_encoder\Encoder.h" // reaponta para a encoder.h no seu pc
#include<stdio.h>
#include<stdlib.h>

/* Used for encoders */
#define INPUT_PULLUP
#define ENCODER_USE_INTERRUPTS


volatile long ticks = 0;

int ENCODER_A = 2;
int ENCODER_B = 3;
int SERVO_MOT_PIN = 9;

Encoder enc(ENCODER_A, ENCODER_B);

//138338,0.23,0.20,1.13,6.22
// 139088,0.09,0.10,0.90,2.21
// 139855,0.02,0.20,0.11,2.76

//750
//767

Servo m1;

const int M1_US_MIN = 544;
const int M1_US_MAX = 2500;

void isrA() { if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ticks--; else ticks++; }
void isrB() { if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ticks++; else ticks--; }

int max_sentido_horario = 2000;
int max_sentido_antihorario = 1000;
int parado = 1500;


void setup() {
  Serial.begin(9600);
  //pinMode(ENCODER_A, INPUT_PULLUP);
  //pinMode(ENCODER_B, INPUT_PULLUP);


  //attachInterrupt(digitalPinToInterrupt(ENCODER_A), isrA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_B), isrB, CHANGE);

  m1.attach(SERVO_MOT_PIN, M1_US_MIN, M1_US_MAX);
  delay(2000);



  m1.writeMicroseconds(1400);
}

//long lerTicks() { noInterrupts(); long t = ticks; interrupts(); return t; }

// passa-baixa de primeira ordem
float y = 0;
float passaBaixa(float x, float alfa) { y += alfa * (x - y); return y; }

float u = 0;

float e = 0;
float e_i = 0;

float controlePID(float r, float y) { 
  e = y - r;
  e_i = e_i + e;
  float Kp = 2;
  float Ki = 0.1;

  u = Kp * e + Ki * e_i;
}

float t_start = 0.0;
float t_end = 0.0;

float k = 0.47997;

long prevpos = -999;
unsigned long timer = 0;

long ti = 0;
long tf = 0;

// 1 volta são ~612 pontos 
// 1 volta são 4.4 pi cm
// anda 0.0225 cm por ponto

void loop() {
  long pos = enc.read(); 
  tf = millis();
  
  while (millis() - timer < 50)
    ;
  timer = millis();

  if (prevpos != pos) {
    float distancia = (pos - prevpos) * (4.4 * 3.1415 / 350); // cm
    float tempo = tf - ti; // ms
    tempo /= 1000.0;
    float v = distancia/tempo; // cm / s
    float vf = passaBaixa(v, 0.05);   

    Serial.print(pos);
    Serial.print(',');
    Serial.print(distancia);
    Serial.print(',');
    Serial.print(tempo);
    Serial.print(',');
    Serial.print(v);
    Serial.print(',');  
    Serial.print(vf);
    Serial.println(';');  
    //Serial.print(',');
    prevpos = pos;
    ti = tf;
    


  }
}





// w = 750 graus por segundo
// w = 13.09 rad/s
// Diametro = 4.4 -> raio 2.2
// v = w r = 750/180 pi * 4.4 = 28.8 cm / s
// C = 2 pi r = 4.4pi = 13.82 cm
// tempo de 1 volta = 0.24 s

// k = pi * D / N 
// N = 288
// D = 4.4
// k = 0.47997



// int mode = 0;
// long t = millis();
// long t1 = t;
// int duracao = 500;

// float ticks_start = 0.0;
// float ticks_end = 0.0;
// float t_start = 0.0;
// float t_end = 0.0;

// float k = 0.47997;

// float velocidade_filtrada = 0.0;

// float referencia = 0;

// void loop() {

//   ticks_start = lerTicks();
//   t_start = millis();
//   delay(10);
//   ticks_end = lerTicks();
//   t_end = millis();
//   //Serial.print("Velocidade: ");

//   float tempo = (t_end - t_start) / 1000.0; // segundos
//   float ticks = (ticks_end - ticks_start); 
//   float ticks_por_segundo = ticks/tempo;
//   float velocidade = k * ticks_por_segundo; // mm / s

//   //Serial.println(velocidade);
//   //Serial.println(" mm/s");
//   //m1.writeMicroseconds(parado);
//   velocidade_filtrada = velocidade  + 0.99 * (velocidade_filtrada - velocidade); 


//   Serial.print(millis());
//   Serial.print(",");
//   Serial.print(referencia);
//   Serial.print(",");
//   Serial.print(velocidade_filtrada);
//   Serial.println(";");
//   delay(10);

//   if (millis() - t > 10000) {
//     mode = (mode + 1) % 6;
//     t = millis();

//     switch(mode) {
//       case 0:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         referencia = 0.2;
//         //Serial.println("20%");
//         //m1.writeMicroseconds((1-referencia)*500 + max_sentido_antihorario);
//         break;
//       case 1:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         referencia = 0.4;
//         //Serial.println("40%");
//         //m1.writeMicroseconds(0.6*500 + max_sentido_antihorario);
//         break;
//       case 2:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         referencia = 0.6;
//         //Serial.println("60%");
//         //1.writeMicroseconds(0.4*500 + max_sentido_antihorario);
//         break;
//       case 3:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         referencia = 0.8;
//         //Serial.println("80%");
//         //m1.writeMicroseconds(0.2*500 + max_sentido_antihorario);
//         break;
//       case 4:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         referencia = 1.0;
//         //Serial.println("100%");
//         //m1.writeMicroseconds(max_sentido_antihorario);
//         break;
//       default:
//         //m1.writeMicroseconds(parado);
//         //delay(2000);
//         //Serial.println("0%");
//         break;
//     }

//     m1.writeMicroseconds((1-referencia)*500 + max_sentido_antihorario);

//   }

//   // Serial.println(lerTicks());
// }
