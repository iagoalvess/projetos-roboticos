// ATENÇÃO!
// intervalo de velocidade deve ser de -28 a +28



#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

// ---------------------------------------------------------------------------
// TP4 — Encoder e controle PID de velocidade da roda (Arduino Mega)
//
// Comandos pela serial (115200 baud):
//   c -> imprime os ticks desde o último 'c' e zera (medida de N à mão)
//   n -> varredura do neutro (1450..1550 us, passo 10 us)
//   a -> curva em malha aberta (1000..2000 us, passo 50 us)
//   f -> degrau em malha aberta (registro para comparar os filtros)
//   d -> malha fechada: degrau único (sintonia dos ganhos)
//   p -> malha fechada: cinco patamares
//   i -> malha fechada: inversão de sentido
//   s -> para tudo (pulso = neutro)
//   e -> liga/desliga o diagnóstico do encoder: imprime "A B ticks" a cada
//        mudança dos pinos (girando devagar deve aparecer 00 01 11 10)
//   t -> alterna a saída das amostras entre CSV (padrão, para captura.py) e
//        o formato do Teleplot (">v_mm:87.4"), para ver o gráfico ao vivo
//   P<valor>, I<valor>, D<valor> -> troca kp, ki ou kd sem regravar
//                                   (ex.: "P1.5"); imprime os ganhos
//
// Cada amostra sai como uma linha CSV:
//   t_s,ticks,s_mm,v_raw,v_pb,v_mm,sp,pulso_us
// Linhas que começam com '#' são comentários (condição, troca de passo, fim).
// Os scripts em ../analise capturam essas linhas e refazem os gráficos.
// ---------------------------------------------------------------------------

const int ENCODER_A = 2;     // canal A do encoder (pino com interrupção)
const int ENCODER_B = 3;     // canal B do encoder (pino com interrupção)
const int SERVO_MOT_PIN = 9; // sinal do servo

Encoder enc(ENCODER_A, ENCODER_B);
Servo m1;

// ---------------------------------------------------------------------------
// Constantes medidas na bancada
// TODO: substituir N e o neutro pelos valores medidos pela equipe (os atuais
//       são os do mecanismo de referência e servem só de conferência).
// ---------------------------------------------------------------------------
const float D_MM = 45.0;     // diâmetro da roda (mm): estimativa usada em todos os ensaios; a volta no papel deu L = 138,0 mm (D = 43,9), ver relatório
const float N_TICKS = 140.0; // ticks por volta da roda (X4, lido com 'c'): média dos módulos, 5 voltas em cada sentido
const int NEUTRO_US = 1500;  // pulso em que a roda para: centro da banda morta 1470..1530 (achado com 'n')

// Sentido da contagem: pulso abaixo do neutro (lado de 1000 us) dá ticks
// negativos e acima (lado de 2000 us) dá ticks positivos. Conferido com o
// motor no ensaio 'a': com +1, 1000 us saía com velocidade positiva.
// Só troca o sinal dos números; o sentido físico da roda é o do servo.
const int SINAL_ENCODER = -1;

const float K_MM_TICK = PI * D_MM / N_TICKS; // mm por tick

// Período de amostragem / controle
// 250 us não era atingido: a impressão serial de cada amostra leva ~10 ms, então
// o período real virava ~10 ms, com 0 ou 1 tick por amostra. 20 ms cabe a
// impressão do Teleplot e ainda dá 50 pontos/s.
const unsigned long PERIODO_US = 20000UL; // 20 ms

// Limites do pulso
const int PULSO_MIN = 1000;
const int PULSO_MAX = 2000;
const float U_MAX = 1000.0; // correção máxima em torno do neutro (us)

// ---------------------------------------------------------------------------
// Filtros da velocidade
// ---------------------------------------------------------------------------
// alfa = 0,15 a 20 ms tem a mesma constante de tempo (~0,12 s) do alfa = 0,3 a
// 50 ms testado no ensaio 'f'; 0,1 a 20 ms deixaria o filtro mais lento (0,18 s).
const float ALFA = 0.05;   // passa-baixa de primeira ordem
const int JANELA_MM = 5;   // média móvel (amostras)
// No ensaio 'f' os dois filtros tiveram o mesmo ruído e o mesmo atraso; a média
// móvel deu derivada menos ruidosa e chegou a zero mais cedo na parada.
const bool PID_USA_PB = true; // true: PID usa o passa-baixa; false: média móvel

float v_pb = 0;
float passaBaixa(float x) {
  v_pb += ALFA * (x - v_pb);
  return v_pb;
}

float buf_mm[JANELA_MM];
int idx_mm = 0;
float mediaMovel(float x) {
  buf_mm[idx_mm] = x;
  idx_mm = (idx_mm + 1) % JANELA_MM;
  float soma = 0;
  for (int j = 0; j < JANELA_MM; j++) soma += buf_mm[j];
  return soma / JANELA_MM;
}

// ---------------------------------------------------------------------------
// Controlador PID
//   u = kp e + ki ∫e dt − kd dv/dt   (e em mm/s, u em us)
// O derivativo atua sobre a velocidade filtrada; a integral é limitada e só é
// zerada quando o setpoint troca de sentido (ou vai a zero). Numa subida de
// patamar ela é mantida: é ela que segura o pulso, e zerar derrubava a roda
// (ensaio patamares_v1).
// Sintonia (ensaio 'd'): ki = kd = 0, sobe kp até oscilar e volta um pouco;
// depois sobe ki até o erro em regime sumir; kd só se sobrar sobressinal.
// ---------------------------------------------------------------------------
// Resultado (degrau_pid_1 a 8): kp = 4 já oscila; ki = 8 volta a oscilar;
// kd = 0,05 tira o pico inicial, kd = 0,1 só deixa o pulso mais ruidoso.
// Com a velocidade multiplicada por 4,4*pi/140 (~0,0987, unidade ~cm/s), um
// erro de 1 unidade vale ~10 mm/s: kp e ki foram multiplicados por ~10 para
// manter a mesma malha da sintonia (2 us/(mm/s) e 4 us/mm). kd = 0 porque a
// derivada a cada 20 ms é ruidosa demais (a 50 ms já estava no limite).
float KP = 20.0;  // us / unidade de v
float KI = 40.0;  // us / (unidade de v · s)
float KD = 0.0;   // us·s / unidade de v

float integral = 0;
float vf_ant = 0;

void resetPID(float vf) {
  integral = 0;
  vf_ant = vf;
}

float controlePID(float sp, float vf, float dt) {
  float e = sp - vf;

  integral += e * dt;
  if (KI > 0) integral = constrain(integral, -U_MAX / KI, U_MAX / KI);

  float dv = (vf - vf_ant) / dt;
  vf_ant = vf;

  float u = KP * e + KI * integral - KD * dv;
  return constrain(u, -U_MAX, U_MAX);
}

// ---------------------------------------------------------------------------
// Ensaios: sequência de passos {duração, valor}. Em malha aberta o valor é o
// pulso (us); em malha fechada é o setpoint (mm/s).
// ---------------------------------------------------------------------------
struct Passo {
  unsigned long dur_ms;
  float valor;
};

const int MAX_PASSOS = 32;
Passo seq[MAX_PASSOS];
int n_passos = 0;
bool malha_fechada = false;
bool ativo = false;
bool diag_enc = false;
bool teleplot = false;  // 't': amostras no formato do Teleplot em vez de CSV

int passo = -1;
unsigned long t_ini_ms = 0;   // início do ensaio
unsigned long t_passo_ms = 0; // início do passo atual

// Escolhidos a partir da curva em malha aberta: abaixo de ~50 mm/s a roda não
// gira de forma estável, e em neutro ± 250 us (limite de U_MAX) o lado mais
// lento chega a ~137 mm/s.
const float PATAMARES[5] = {60, 75, 90, 105, 120}; // mm/s
const float V_INVERSAO = 100;                        // mm/s
const float V_DEGRAU = 100;                          // mm/s (sintonia)

void carregaNeutro() {
  malha_fechada = false;
  n_passos = 0;
  // A busca é em torno de 1500 us, não do NEUTRO_US já gravado.
  for (int p = 1450; p <= 1550; p += 10)
    seq[n_passos++] = {2000, (float)p};
}

void carregaAberta() {
  malha_fechada = false;
  n_passos = 0;
  for (int p = PULSO_MIN; p <= PULSO_MAX; p += 50)
    seq[n_passos++] = {5000, (float)p};
}

void carregaDegrau() {
  malha_fechada = false;
  n_passos = 0;
  seq[n_passos++] = {1000, (float)NEUTRO_US};
  seq[n_passos++] = {4000, (float)(NEUTRO_US + 150)};
  seq[n_passos++] = {2000, (float)NEUTRO_US};
}

void carregaDegrauFechado() {
  malha_fechada = true;
  n_passos = 0;
  seq[n_passos++] = {1000, 0};
  seq[n_passos++] = {4000, V_DEGRAU/8.0};
  seq[n_passos++] = {1000, 0};
}

void carregaPatamares() {
  malha_fechada = true;
  n_passos = 0;
  seq[n_passos++] = {1000, 0};
  for (int j = 0; j < 5; j++) seq[n_passos++] = {3000, PATAMARES[j]};
  seq[n_passos++] = {2000, 0};
}

void carregaInversao() {
  malha_fechada = true;
  n_passos = 0;
  seq[n_passos++] = {1000, 0};
  seq[n_passos++] = {4000, V_INVERSAO};
  seq[n_passos++] = {4000, -V_INVERSAO};
  seq[n_passos++] = {2000, 0};
}

// ---------------------------------------------------------------------------
// Estado da medição
// ---------------------------------------------------------------------------
long ticks_ant = 0;
unsigned long t_amostra_us = 0;
int pulso = NEUTRO_US;

long leTicks() {
  return SINAL_ENCODER * enc.read();
}

void escrevePulso(int p) {
  pulso = constrain(p, PULSO_MIN, PULSO_MAX);
  m1.writeMicroseconds(pulso);
}

void imprimeGanhos() {
  Serial.print(F("# kp="));
  Serial.print(KP, 4);
  Serial.print(F(" ki="));
  Serial.print(KI, 4);
  Serial.print(F(" kd="));
  Serial.println(KD, 4);
}

void imprimePasso() {
  Serial.print(F("# passo="));
  Serial.print(passo);
  Serial.print(malha_fechada ? F(" sp_mm_s=") : F(" pulso_us="));
  Serial.println(seq[passo].valor, 1);
}

void imprimeCondicao(const char *nome) {
  Serial.print(F("# ensaio="));
  Serial.print(nome);
  Serial.print(F(" neutro_us="));
  Serial.print(NEUTRO_US);
  Serial.print(F(" D_mm="));
  Serial.print(D_MM, 2);
  Serial.print(F(" N="));
  Serial.print(N_TICKS, 0);
  Serial.print(F(" k_mm_tick="));
  Serial.print(K_MM_TICK, 4);
  Serial.print(F(" sinal_enc="));
  Serial.print(SINAL_ENCODER);
  Serial.print(F(" dt_ms="));
  Serial.print(PERIODO_US / 1000);
  Serial.print(F(" alfa="));
  Serial.print(ALFA, 3);
  Serial.print(F(" janela="));
  Serial.print(JANELA_MM);
  Serial.print(F(" filtro_pid="));
  Serial.print(PID_USA_PB ? F("passa-baixa") : F("media-movel"));
  Serial.print(F(" kp="));
  Serial.print(KP, 4);
  Serial.print(F(" ki="));
  Serial.print(KI, 4);
  Serial.print(F(" kd="));
  Serial.println(KD, 4);
  Serial.println(F("t_s,ticks,s_mm,v_raw,v_pb,v_mm,sp,pulso_us"));
}

void iniciaEnsaio(const char *nome) {
  enc.write(0);
  ticks_ant = 0;
  v_pb = 0;
  for (int j = 0; j < JANELA_MM; j++) buf_mm[j] = 0;
  resetPID(0);

  imprimeCondicao(nome);
  ativo = true;
  passo = 0;
  imprimePasso();
  t_ini_ms = millis();
  t_passo_ms = t_ini_ms;
  t_amostra_us = micros();
}

void paraEnsaio() {
  ativo = false;
  passo = -1;
  escrevePulso(NEUTRO_US);
  Serial.println(F("# fim"));
}

void trataSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'c': {
      long t = leTicks();
      Serial.print(F("# ticks="));
      Serial.print(t);
      Serial.print(F(" s_mm="));
      Serial.println(K_MM_TICK * t, 2);
      enc.write(0);
      break;
    }
    case 'n': carregaNeutro();        iniciaEnsaio("neutro");         break;
    case 'a': carregaAberta();        iniciaEnsaio("malha_aberta");   break;
    case 'f': carregaDegrau();        iniciaEnsaio("degrau_filtros"); break;
    case 'd': carregaDegrauFechado(); iniciaEnsaio("degrau_pid");     break;
    case 'p': carregaPatamares();     iniciaEnsaio("patamares");      break;
    case 'i': carregaInversao();      iniciaEnsaio("inversao");       break;
    case 's': paraEnsaio(); break;
    case 'e':
      diag_enc = !diag_enc;
      Serial.println(diag_enc ? F("# diag encoder: A B ticks") : F("# diag encoder off"));
      break;
    case 't':
      teleplot = !teleplot;
      Serial.println(teleplot ? F("# saida: teleplot") : F("# saida: csv"));
      break;
    // Troca de ganho só fora de um ensaio, para o registro não misturar ganhos.
    case 'P': if (!ativo) KP = Serial.parseFloat(); imprimeGanhos(); break;
    case 'I': if (!ativo) KI = Serial.parseFloat(); imprimeGanhos(); break;
    case 'D': if (!ativo) KD = Serial.parseFloat(); imprimeGanhos(); break;
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  m1.attach(SERVO_MOT_PIN, PULSO_MIN, PULSO_MAX);
  escrevePulso(NEUTRO_US);
  Serial.println(F("# pronto: c n a f d p i s P I D"));
}

// Mostra o nível bruto dos canais A e B só quando algum deles muda.
void diagnosticoEncoder() {
  static int ab_ant = -1;
  int ab = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  if (ab == ab_ant) return;
  ab_ant = ab;
  Serial.print(ab >> 1);
  Serial.print(' ');
  Serial.print(ab & 1);
  Serial.print(' ');
  Serial.println(leTicks());
}

// Formato do Teleplot (extensão do VS Code): uma linha ">nome:valor" por variável.
// O Teleplot ajusta o eixo aos dados, então v_lim_max e v_lim_min (constantes)
// fixam a escala em ±V_ESCALA: neutro ± 250 us dá +142 e -143 mm/s na curva
// aberta, e com essas linhas no mesmo gráfico o setpoint não muda de posição.
const float V_ESCALA = 150; // mm/s

void imprimeTeleplot(float v, float vpb, float vmm, float sp, float s) {
  Serial.print(F(">v_lim_max:")); Serial.println(V_ESCALA, 0);
  Serial.print(F(">v_lim_min:")); Serial.println(-V_ESCALA, 0);
  Serial.print(F(">v_raw:"));  Serial.println(v, 1);
  Serial.print(F(">v_pb:"));   Serial.println(vpb, 2);
  Serial.print(F(">v_mm:"));   Serial.println(vmm, 1);
  Serial.print(F(">sp:"));     Serial.println(sp, 1);
  Serial.print(F(">pulso:"));  Serial.println(double(pulso), 2);
  Serial.print(F(">s_mm:"));   Serial.println(s, 1);
}

void loop() {
  trataSerial();
  if (diag_enc) diagnosticoEncoder();
  if (!ativo) return;

  unsigned long agora_us = micros();
  if (agora_us - t_amostra_us < PERIODO_US) return;

  // Δt real entre as leituras
  float dt = (agora_us - t_amostra_us) / 1e6;
  t_amostra_us = agora_us;

  long ticks = leTicks();
  float s = K_MM_TICK * ticks;
  float v = K_MM_TICK * (ticks - ticks_ant) / dt * (4.4 * 3.1415 / 140); // mm/s
  ticks_ant = ticks;

  float vpb = passaBaixa(v);
  float vmm = mediaMovel(v);

  // Avança a sequência do ensaio
  unsigned long agora_ms = millis();
  if (agora_ms - t_passo_ms >= seq[passo].dur_ms) {
    passo++;
    t_passo_ms = agora_ms;
    if (passo >= n_passos) {
      paraEnsaio();
      return;
    }
    imprimePasso();
    if (malha_fechada && seq[passo].valor * seq[passo - 1].valor <= 0)
      resetPID(PID_USA_PB ? vpb : vmm);
  }

  float sp = 0;
  if (malha_fechada) {
    sp = seq[passo].valor;
    float vf = PID_USA_PB ? vpb : vmm;
    // if (sp == 0) {
    //   // Setpoint nulo: roda parada no neutro, sem o PI caçar a banda morta.
    //   resetPID(vf);
    //   escrevePulso(NEUTRO_US);
    // } else {
      float u = controlePID(sp, vf, dt);
      escrevePulso(NEUTRO_US + (int)u);
    // }
  } else {
    escrevePulso((int)seq[passo].valor);
  }

  if (teleplot) {
    imprimeTeleplot(v, vpb, vmm, sp, s);
    return;
  }

  Serial.print((agora_ms - t_ini_ms) / 1000.0, 3);
  Serial.print(',');
  Serial.print(ticks);
  Serial.print(',');
  Serial.print(s, 2);
  Serial.print(',');
  Serial.print(v, 2);
  Serial.print(',');
  Serial.print(vpb/240, 2);
  Serial.print(',');
  Serial.print(vmm, 2);
  Serial.print(',');
  Serial.print(sp, 1);
  Serial.print(',');
  Serial.println((pulso - 1500)*0.002);
}
