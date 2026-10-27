# TP4 — Encoder e controle PID de velocidade

Arduino Mega (com Motor Shield v2 empilhado) + MG90S de giro contínuo + encoder mecânico.

## Ligação

| Fio | Pino da Mega |
|---|---|
| Servo (sinal) | D9 (conector *Servo 2* do shield); alimentação por fonte externa, GND comum |
| Encoder, terminal da ponta (canal A) | D2 |
| Encoder, terminal da ponta (canal B) | D3 |
| Encoder, terminal do **meio** (comum) | **GND** |

O encoder não é alimentado: os canais usam o pull-up interno. D0 e D1 ficam livres (serial da USB).
A biblioteca `Encoder.h` foi alterada para ignorar transições em que A e B mudam juntos
(falha do contato do comum), e com `SINAL_ENCODER = -1` o lado de 1000 µs conta negativo (conferido com o motor no ensaio `a`).
Detalhes na seção "Verificação e correção da contagem do encoder" do relatório.

## Estrutura

| Caminho | Conteúdo |
|---|---|
| `servo_e_encoder/` | firmware (projeto PlatformIO, `env:mega`) |
| `analise/captura.py` | dispara um ensaio pela serial e grava `dados/<ensaio>.csv` |
| `analise/graficos.py` | refaz figuras (`relatorio/relatorio-4/imagens/`) e métricas (`dados/resumo.txt`) |
| `dados/` | registros dos ensaios (CSV) |
| `relatorio/relatorio-4.tex` | relatório (Overleaf; usa o `template/` compartilhado) |
| `velocidade*.txt` | registros preliminares do firmware antigo (só sentido positivo) |

## Compilar e gravar

```sh
cd servo_e_encoder
pio run -t upload      # compila e grava
pio device monitor     # 115200 baud
```

## Ensaios

| Comando | Ensaio | Arquivo |
|---|---|---|
| `c` | imprime e zera os ticks (girar a roda uma volta à mão → N) | — |
| `n` | busca do neutro, 1450–1550 µs, passo 10 µs | `neutro.csv` |
| `a` | malha aberta, 1000–2000 µs, passo 50 µs | `malha_aberta.csv` |
| `f` | degrau em malha aberta (comparação dos filtros) | `degrau_filtros.csv` |
| `d` | degrau em malha fechada (sintonia) | `degrau_pid*.csv` |
| `p` | cinco patamares | `patamares.csv` |
| `i` | inversão de sentido | `inversao.csv` |
| `P<v>` `I<v>` `D<v>` | troca kp, ki, kd sem regravar (ex.: `P1.5`) | — |
| `s` | para e aplica o neutro | — |
| `e` | liga/desliga o diagnóstico do encoder (`A B ticks` a cada mudança dos pinos) | — |
| `t` | alterna a saída entre CSV e o formato do Teleplot (`>v_mm:87.4`), para ver ao vivo | — |

```sh
pip install pyserial numpy matplotlib
cd analise
python captura.py n
python captura.py a
python captura.py f
python captura.py d --nome degrau_pid_1 --ganhos P1 I0 D0   # uma por tentativa
python captura.py p
python captura.py i
python graficos.py
```

Cada CSV começa com a linha `# ensaio=... neutro_us=... k_mm_tick=... dt_ms=... alfa=... janela=... kp=... ki=... kd=...`
e tem as colunas `t_s,ticks,s_mm,v_raw,v_pb,v_mm,sp,pulso_us`.

## Constantes da bancada


| D (mm) | N (ticks/volta) | k (mm/tick) | neutro (µs) | α | janela | kp | ki | kd |
|---|---|---|---|---|---|---|---|---|
| 45,0 (estimativa usada nos ensaios; volta no papel: L = 138,0 mm → D = 43,9) | 140 (N− = −143,2; N+ = 136,4) | 1,010 | 1500 (banda morta 1470–1530) | 0,3 | 5 (usada no PID) | 2 | 4 | 0,05 |

## Passo a passo

Feito:

- [x] D = 45,0 mm nos ensaios; volta no papel L = 138, 137, 139 mm → D = 43,9 mm (k real ≈ 2,4% menor; ensaios não refeitos).
- [x] Ligação do encoder corrigida (comum no GND) e decodificação ajustada no `Encoder.h`.
- [x] Sentido da contagem: lado de 1000 µs negativo, lado de 2000 µs positivo (`SINAL_ENCODER = -1`, conferido com o motor).
- [x] N = 140 ticks/volta (`N_TICKS`), k = 1,010 mm/tick.
- [x] Neutro = 1500 µs (banda morta 1470–1530 µs; a roda parte a ~40–60 mm/s) — `dados/neutro.csv`.
- [x] Malha aberta (`dados/malha_aberta.csv`): ~0,4 mm/s por µs, sem saturação até ±500 µs,
      ~142 mm/s em neutro ± 250 µs. `PATAMARES = {60, 75, 90, 105, 120}`, `V_INVERSAO = V_DEGRAU = 100`.
- [x] Filtros (`dados/degrau_filtros.csv`): mesmo ruído (~7 mm/s) e atraso (t63 = 0,25 s);
      PID usa a média móvel, M = 5 (`PID_USA_PB = false`).
- [x] Sintonia (`dados/degrau_pid_1` a `8`): kp = 2, ki = 4, kd = 0,05, gravados em `KP`, `KI`, `KD`.
      Só P: kp = 4 já oscila. PI: ki = 2 lento, ki = 8 oscila, ki = 4 com pico de 29%.
      kd = 0,05 tira o pico; kd = 0,1 só deixa o pulso ruidoso.
- [x] Inversão (`dados/inversao.csv`): ±100 mm/s com erro < 6%, pulso no limite por 0,2 s na troca.
- [x] Patamares (`dados/patamares.csv`): erro ≤ 3,5 mm/s, novo setpoint em ≤ 0,35 s; distância 1322 mm (−2,5%).
      O primeiro ensaio (`dados/patamares_v1.csv`) zerava a integral a cada patamar e derrubava a roda;
      o firmware agora só zera na troca de sentido ou no setpoint zero.

Pendências do relatório: nenhuma (`\todo` resolvidos).

