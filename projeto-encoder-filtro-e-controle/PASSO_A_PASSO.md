# TP4 — Passo a passo (bancada → relatório → entrega)

Entrega: **03/11/2026** (PDF no Moodle + link do repositório).

Convenções:
- Comandos de terminal partem da pasta `projeto-encoder-filtro-e-controle/`.
- Linhas citadas do firmware são de `servo_e_encoder/servo_e_encoder.ino`; as do relatório, de `relatorio/relatorio-4.tex`.
- **Regravar o firmware** = `cd servo_e_encoder && pio run -t upload`.
- Feche o `pio device monitor` antes de usar o `captura.py`: os dois não abrem a porta ao mesmo tempo.

Uma vez só:

```sh
pip install pyserial numpy matplotlib
```

---

## Parte A — Bancada

### A1. Montagem e fotos

1. [ ] Montar as duas transmissões: pinhão do SG90 → engrenagem da roda; engrenagem solidária à roda → encoder. Conferir que não há folga nem dente pulando.
2. [ ] Ligar: servo no **D9**, encoder A/B em **D2/D3**, 5 V e **GND comum**.
3. [ ] **Fotografar antes de fechar o conjunto** e salvar em `relatorio/relatorio-4/imagens/`:
   - `foto_transmissao` — engrenagens visíveis (foto do conjunto fechado não substitui);
   - `foto_roda` — roda montada no chassi;
   - `foto_ligacao` — ligação na Mega, fios identificáveis.
4. [ ] Gravar o firmware:
   ```sh
   cd servo_e_encoder
   pio run -t upload
   ```

### A2. Ticks por volta (N)

5. [ ] Abrir o monitor (`pio device monitor`, 115200 baud). Deve aparecer `# pronto: c n a f d p i s P I D`.
6. [ ] Enviar `c` (zera). Marcar a roda, girar **uma volta inteira à mão** no sentido +, enviar `c` → anotar `ticks` (N+).
7. [ ] Repetir no sentido − → anotar N−. Se os módulos diferirem, anotar a causa provável (folga, volta imprecisa) e qual valor será adotado.
8. [ ] Repetir 2–3 vezes por sentido para ter uma média.

| Tentativa | N+ | N− |
|---|---|---|
| 1 | | |
| 2 | | |
| 3 | | |

### A3. Neutro

9. [ ] Fechar o monitor e rodar:
   ```sh
   cd analise
   python captura.py n
   ```
   (1450–1550 µs, passo 10 µs, 2 s cada, ~22 s)
10. [ ] `python graficos.py` e ler em `dados/resumo.txt` / figura qual pulso deixa a roda parada.
11. [ ] Gravar no firmware `N_TICKS` (linha 39) e `NEUTRO_US` (linha 40) → **regravar o firmware**.
12. [ ] Calcular:
    - k = π · 45,0 / N = ______ mm/tick
    - q = k / 0,050 s = ______ mm/s por tick (degrau da velocidade medida)

### A4. Malha aberta

13. [ ] `python captura.py a` (1000–2000 µs, passo 50 µs, 3 s cada, ~63 s).
14. [ ] `python graficos.py` e abrir a curva pulso × velocidade.
15. [ ] Conferir o sentido: pulso **acima** do neutro deve dar velocidade **positiva**. Se não der: trocar A e B **ou** `SINAL_ENCODER = -1` (linha 45) → regravar → refazer o passo 13. Anotar isso para o relatório.
16. [ ] Anotar da curva:
    - banda morta: de ______ a ______ µs;
    - saturação: acima de ______ µs / abaixo de ______ µs;
    - velocidade máxima: + ______ mm/s, − ______ mm/s (assimetria).
17. [ ] Escolher, dentro de neutro ± 250 µs (linhas 136–138):
    - `PATAMARES` — 5 valores crescentes, o menor sendo o mais lento que a malha ainda segura;
    - `V_INVERSAO` e `V_DEGRAU`.
18. [ ] **Regravar o firmware.**

### A5. Filtros

19. [ ] `python captura.py f` (degrau aberto neutro → neutro+150 µs → neutro, 7 s).
20. [ ] `python graficos.py`: comparar bruta × passa-baixa (α) × média móvel (M).
21. [ ] Se preciso, ajustar `ALFA` / `JANELA_MM` (linhas 60–61) e refazer 19–20.
22. [ ] Escolher o filtro do PID em `PID_USA_PB` (linha 62) e anotar **por quê** (ruído × atraso). **Regravar o firmware.**

### A6. Sintonia do PID

Um ganho por vez, sempre olhando o gráfico do degrau (`graficos.py` depois de cada tentativa). Numerar os arquivos.

23. [ ] Só kp (ki = kd = 0), subindo até oscilar:
    ```sh
    python captura.py d --nome degrau_pid_1 --ganhos P1 I0 D0
    python captura.py d --nome degrau_pid_2 --ganhos P2 I0 D0
    ...
    ```
    Depois voltar para um kp menor que o de oscilação.
24. [ ] Subir ki até o erro em regime sumir; parar quando o sobressinal incomodar.
25. [ ] kd só se ainda sobrar sobressinal (em geral um PI basta).
26. [ ] Preencher a tabela a cada tentativa (vai direto para o relatório):

| # | Kp [µs/(mm/s)] | Ki [µs/mm] | Kd [µs·s/mm] | O que o gráfico mostrou | Decisão |
|---|---|---|---|---|---|
| 1 | | 0 | 0 | | |
| 2 | | 0 | 0 | | |
| 3 | | 0 | 0 | | |
| 4 | | | 0 | | |
| 5 | | | | | ganhos finais |

27. [ ] Gravar os ganhos finais em `KP`, `KI`, `KD` (linhas 89–91) → **regravar o firmware** (ganhos trocados pela serial se perdem no reset).

### A7. Ensaios finais (mesmos ganhos e mesmo filtro)

28. [ ] Cinco patamares: `python captura.py p` (~18 s).
29. [ ] Inversão: `python captura.py i` (~11 s). Conferir que o sinal da velocidade bate com o sentido real da roda.
30. [ ] Distância (opcional, mas ajuda na discussão): marcar a roda e contar as voltas durante `p` e `i`, ou medir contra uma marca no chão. Anotar: ______ voltas → πD × voltas = ______ mm.
31. [ ] `python graficos.py` → conferir que todas as figuras foram geradas em `relatorio/relatorio-4/imagens/` e que `dados/resumo.txt` está completo.

Se algum ensaio sair ruim (fio solto, roda travou), refazer antes de desmontar a bancada.

---

## Parte B — Relatório (`relatorio/relatorio-4.tex`)

32. [ ] **Fotos** nas figuras (linhas ~106, 115, 121).
33. [ ] **Tabela de constantes** (~157–167): N+, N−, k, Δt, q, neutro, α, M, Kp, Ki, Kd — com unidades.
34. [ ] **Modelo do encoder** (~146, 202): método de medida de D (volta no papel, 45,0 mm), k medido × referência (49,2 mm / 288 ticks), q com o próprio k e o que isso faz em velocidade baixa.
35. [ ] **Neutro** (~172–177): figura da busca e valor escolhido.
36. [ ] **Malha aberta** (~315–321): figura com o neutro marcado; comentar banda morta, saturação e assimetria.
37. [ ] **Filtros** (~220, 335–354): figura (bruta + os dois filtros), tabela de ruído/atraso, comentário e justificativa da escolha (efeito no derivativo/sobressinal).
38. [ ] **PID e sintonia** (~277, 362–384): lei de controle, limites do pulso (1000–2000 µs, ±250 µs), tabela de tentativas, figura dos degraus, ganhos finais com unidades.
39. [ ] **Cinco patamares** (~388–415): setpoints reais, tabela por patamar; comentar tempo de subida, sobressinal, erro residual e se a integral fica para trás na troca.
40. [ ] **Inversão** (~419–428): figura; comentar banda morta na passagem pelo zero, tempo até o novo setpoint, saturação, sentido do encoder (e se A/B foram trocados).
41. [ ] **Distância** (~441): s final nos dois ensaios, número de voltas, comparação com πD × voltas ou marca no chão.
42. [ ] **Discussão** (~445–455), **mínimo 1 página**: o que a curva aberta explica nos degraus, o que o filtro atrasou, onde saturou nos ±250 µs, o que não ficou bom.
43. [ ] **Conclusão** (~463): ganhos, filtro, faixa de velocidades que a roda acompanha.
44. [ ] **Introdução** (~71): conferir que descreve o que foi entregue de fato.
45. [ ] Procurar `\todo` até não sobrar nenhum.
46. [ ] Compilar e conferir:
    - 10 a 16 páginas;
    - todo gráfico de malha fechada mostra o **setpoint**;
    - velocidade em **mm/s** (nunca ticks/s);
    - neutro com o valor **medido** (não 1500 µs sem a busca);
    - cada ensaio declara neutro, k, Δt, ganhos e parâmetro do filtro.

---

## Parte C — Entrega

47. [ ] Preencher a tabela "Constantes da bancada" do `README.md` e remover a seção "O que falta".
48. [ ] Commit e push: firmware, `dados/*.csv`, fotos, figuras, `.tex`, README.
49. [ ] Enviar o PDF no Moodle **com o link do repositório** até 03/11/2026.

---

## Referência rápida — comandos seriais

| Cmd | O que faz |
|---|---|
| `c` | imprime e zera os ticks (medir N à mão) |
| `n` | busca do neutro, 1450–1550 µs |
| `a` | malha aberta, 1000–2000 µs |
| `f` | degrau aberto para comparar filtros |
| `d` | degrau fechado (sintonia) |
| `p` | cinco patamares |
| `i` | inversão de sentido |
| `s` | para e volta ao neutro |
| `P<v>` `I<v>` `D<v>` | troca kp/ki/kd (só fora de um ensaio; ex.: `P1.5`) |

`p` minúsculo = patamares; `P` maiúsculo = kp.
