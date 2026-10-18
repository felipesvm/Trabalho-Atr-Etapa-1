# Logística colaborativa com robôs móveis autônomos — Etapa 1

Trabalho Final de **Automação em Tempo Real 2026/2** (UFMG — Escola de Engenharia — DELT), prof. Leandro Freitas.

| Integrante | Matrícula |
|---|---|
| Felipe Santos Souza | 2021034652 |
| Davi Assis Silveira | 2022042632 |
| Gabriel Caldeira | 2022061645 |

Este repositório contém a implementação da **Etapa 1**: as tarefas em C++17 (Gestor de Robôs, Comando de Navegação, Status do Robô, Organizador da Logística e Coletor de Dados), a troca de informações entre elas por IPC e a criação automática de um processo por robô. O broker MQTT, o simulador e a tela de operação ficam para a Etapa 2; até lá, o papel deles é feito por um driver de E/S local e por ferramentas de linha de comando.

---

## Sumário

1. [O que o sistema faz](#1-o-que-o-sistema-faz)
2. [Arquitetura](#2-arquitetura)
3. [Comunicação entre as tarefas](#3-comunicação-entre-as-tarefas)
4. [Mecanismos de sincronização](#4-mecanismos-de-sincronização)
5. [Ciclo de vida de uma carga](#5-ciclo-de-vida-de-uma-carga)
6. [Compilação e execução](#6-compilação-e-execução)
7. [Cenário](#7-cenário)
8. [Ferramentas](#8-ferramentas)
9. [Testes](#9-testes)
10. [Organização do código](#10-organização-do-código)
11. [Limitações conhecidas e próximos passos](#11-limitações-conhecidas-e-próximos-passos)

---

## 1. O que o sistema faz

Um galpão recebe caixas nas baias. Cada caixa tem um tamanho de 1 a 4, que é o número de robôs necessários para carregá-la. O sistema:

- escolhe a próxima caixa de cada baia e forma uma equipe com o número exato de robôs (tudo ou nada);
- leva cada robô até o seu ponto de acoplamento ao redor da carga;
- só libera a carga quando todos os membros estão em posição, e faz todos acoplarem no mesmo instante;
- transporta a carga em formação até a zona de destino, respeitando passagens estreitas que comportam 1, 2 ou 3 robôs por vez;
- desacopla a equipe ao mesmo tempo no destino e registra a entrega com data e hora;
- reage a defeitos (o robô para no ciclo seguinte e a equipe deposita a carga), a bateria baixa ou crítica (recarga), à perda de comunicação com o hardware e a comandos do operador.

O tamanho da frota é **parâmetro de execução**: `./run.sh --robos N`. Nenhum valor de N aparece no código, e robôs novos podem ser anunciados com o sistema rodando.

## 2. Arquitetura

```
                        ┌──────────────────────────────┐
  anunciar_robo ──────▶ │ amr_gestor                   │ fork + execv ─────────┐
  (fila de teste)       │ T_GES_ANUNCIO  T_GES_SPAWN   │                       │
                        │ T_GES_REAP (waitpid)         │                       ▼
                        └──────────────┬───────────────┘        ┌──────────────────────────┐
                                       │ NOVO_ROBO,             │ amr_robo  (× N)          │
                                       │ ROBO_TERMINOU          │ T_NAV    50 ms  P80      │
                                       ▼                        │ T_STATUS 100 ms P75      │
┌─────────────────────────────┐   /amr_org_inbox  ◀──────────── │ T_IO (DriverStub)  P70   │
│ amr_organizador (dono do IPC)│◀── eventos ──────────────────── │ T_ROTA (A*)        P40   │
│ T_ORG_EVT   evento   P65     │                                 └───┬──────────┬──────────┘
│ T_ORG_DESP  200 ms   P60     │── /amr_cmd_<id> (missões) ────────▶ │          │
│ T_ORG_SUP   500 ms   P55     │                                     │ status   │ registros
└──────────────┬──────────────┘                                     ▼          ▼
               │ lê status                       ┌─────────────────────┐  ┌────────────────────┐
               └───────────────────────────────▶ │ /amr_frota (shm)    │  │ /amr_eventos (shm) │
                                                 │ slots de robô,      │  │ buffer circular    │
   semáforos System V: 1 por passagem ◀── T_NAV  │ equipes, passagens  │  │ 1024 registros     │
                                                 └─────────────────────┘  └─────────┬──────────┘
                                                                                    ▼
                                                              ┌──────────────────────────────┐
                                                              │ amr_coletor                  │
                                                              │ T_COL_CONS  T_COL_SNAP (1 s) │
                                                              │ logs/<data>/*.csv            │
                                                              └──────────────────────────────┘
```

### Processos

| Processo | Instâncias | Criado por | Papel |
|---|---|---|---|
| `amr_organizador` | 1 | `run.sh` | Cria e remove todos os objetos de IPC; despacho, reunião das equipes, supervisão. |
| `amr_coletor` | 1 | `run.sh` | Consome o buffer de eventos e grava em disco, com data e hora. |
| `amr_gestor` | 1 | `run.sh` | Recebe anúncios de robôs, cria um processo por robô e detecta o término de cada um. |
| `amr_robo` | N | `amr_gestor` | Navegação, status, planejamento de rota e E/S de um robô. |

### Threads e requisitos temporais

| Thread | Processo | Ativação | Prioridade `SCHED_FIFO` | Função |
|---|---|---|---|---|
| `T_NAV` | robô | 50 ms | 80 | Setpoints, passagens, formação, acoplamento no instante `t_go`. Nunca bloqueia. |
| `T_STATUS` | robô | 100 ms | 75 | Defeito, bateria, comunicação, distância, publicação do slot e *heartbeat*. |
| `T_IO` | robô | 50 ms ou saída nova | 70 | `DriverStub`: cinemática local, bateria, atracamento, injeções de teste. |
| `T_ROTA` | robô | evento | 40 | A* com a trava solta; pedidos sucessivos são fundidos. |
| `T_ORG_EVT` | organizador | evento | 65 | Consome `/amr_org_inbox`, conta chegadas, define `t_go`. |
| `T_ORG_DESP` | organizador | 200 ms | 60 | Abastece as baias, forma equipes, envelhecimento e reserva. |
| `T_ORG_SUP` | organizador | 500 ms | 55 | *Watchdog*, prazos de reunião (60 s) e acoplamento (3 s), alarmes. |
| `T_GES_ANUNCIO`, `T_GES_SPAWN`, `T_GES_REAP` | gestor | evento | normal | Anúncio, `fork`+`execv`, `waitpid`. |
| `T_COL_CONS`, `T_COL_SNAP` | coletor | evento / 1 s | normal | Gravação dos eventos e amostra da frota. |

Todas as tarefas periódicas usam a classe `TarefaPeriodica` (`src/comum/tarefa_periodica.*`): liberação em instante absoluto com `clock_nanosleep(TIMER_ABSTIME)`, medição do tempo de execução e do atraso de liberação, e descarte das liberações perdidas (não executa ciclos em rajada). Três perdas seguidas na `T_NAV` congelam o setpoint e geram `DEADLINE_NAV`. As métricas vão para o slot de cada robô e aparecem no `amr_monitor` e em `frota.csv`.

Cada processo separa **domínio** (classes sem threads nem IPC, como o seguidor de waypoints, o A*, o monitor de bateria e o despachante) do **contexto de execução** (as threads, que temporizam, travam, copiam e chamam o domínio). O domínio é coberto pelos testes unitários.

## 3. Comunicação entre as tarefas

A regra é: **estado** vai em memória compartilhada (vale o último valor), **evento** vai em fila (nenhum pode se perder).

| Objeto | Tipo | Quem escreve → quem lê | Conteúdo |
|---|---|---|---|
| `/amr_frota` | memória compartilhada POSIX | `T_STATUS` de cada robô → organizador, coletor, monitor | Cabeçalho (N_max, `parada_geral` atômica), N_max slots de robô, N_max slots de equipe e o espelho das passagens. |
| `/amr_cmd_<id>` | fila POSIX, uma por robô | organizador → `T_NAV` | `PARAR`, `ABORTAR`, `ENCERRAR` (prioridade 31), `RETOMAR`, `MODO` (20), `DESTINO_MANUAL` (15), `MISSAO_EQUIPE`, `RECONHECER_FALHA` (10). |
| `/amr_org_inbox` | fila POSIX | robôs, gestor e ferramentas → `T_ORG_EVT` | `FALHA`, `BATERIA_*`, `NOVO_ROBO`, `ROBO_PRONTO`, `ROBO_TERMINOU`, `EM_POSICAO`, `ATRACADO`, `ENTREGUE`, `ABORTO_CONCLUIDO`, comandos do operador… |
| semáforos System V | conjunto com um semáforo por passagem | `T_NAV` dos robôs | Vagas livres em cada passagem estreita. |
| `/amr_eventos` + `/amr_ev_vazios`, `/amr_ev_itens` | buffer circular em shm + semáforos POSIX nomeados | todos → coletor | Registros de rastreabilidade (carga, robô, sistema, operador). |
| `/amr_teste_anuncio`, `/amr_teste_hw_<id>` | filas POSIX de teste | ferramentas → gestor / `T_IO` | Anúncio de robô; defeito, bateria e comunicação injetados. Somente na Etapa 1. |

Todas as mensagens têm 128 bytes, cabeçalho comum (`tipo`, `versao`, `origem`, `seq`, `t_mono_ns`) e corpo específico (`src/comum/protocolo.hpp`). O leiaute da memória compartilhada está em `src/comum/frota.hpp`.

As filas são criadas com 64 (entrada do organizador) e 16 mensagens (comandos). Se o sistema não permitir passar de `fs.mqueue.msg_max` (10 por padrão no Linux), elas são criadas com 10; todo produtor trata fila cheia guardando a mensagem e tentando de novo no ciclo seguinte.

## 4. Mecanismos de sincronização

| Problema | Mecanismo | Onde |
|---|---|---|
| Passagem com capacidade *c* | Semáforo de contagem System V com valor inicial *c*. Robô sozinho toma 1 vaga; equipe de *k* toma *k* de uma vez, com `semop(-k, IPC_NOWAIT)`. Passagens consecutivas são adquiridas na mesma chamada. Sem `SEM_UNDO`: robô morto continua fisicamente na passagem. | `navegacao.cpp` (`tentar_passagens`, `liberar_passagens`) |
| Inanição na passagem | Quem espera há mais de 2 s fica registrado no espelho da passagem e os demais cedem a vez; depois de 20 s o robô pede uma rota alternativa. | `navegacao.cpp` |
| Carga só liberada com todos em posição | *Rendezvous* no organizador: cada membro envia `EM_POSICAO`; com *k* chegadas a equipe passa a `REUNIDA`. | `organizador.cpp` |
| Acoplamento síncrono | Instante comum: o organizador grava `t_go = agora + 100 ms` (relógio monotônico) no slot da equipe, e cada `T_NAV` aciona `o_acoplar_e_erguer` no primeiro ciclo com `t ≥ t_go`. O mesmo vale para o desacoplamento e para o aborto. | `navegacao.cpp` (`ciclo_equipe`) |
| Deslocamento em formação | Barreira não bloqueante por waypoint: cada membro marca seu bit na máscara de chegada; o líder só libera o próximo passo com a máscara completa. | `navegacao.cpp` (`transportar`) |
| Status de cada robô | Um mutex por slot, `PTHREAD_PROCESS_SHARED` + `PTHREAD_MUTEX_ROBUST` + `PTHREAD_PRIO_INHERIT`. Se o dono morre com a trava, o próximo `lock` recebe `EOWNERDEAD`, recupera o mutex e segue. | `ipc.cpp` (`TravaCompartilhada`) |
| Eventos para o coletor | Produtor–consumidor clássico: `sem_vazios`, `sem_itens` e mutex do buffer. Robôs usam `sem_trywait` (contam a perda e seguem); organizador e gestor usam `sem_timedwait` de 10 ms. | `eventos.cpp`, `coletor.cpp` |
| Imagem de E/S do robô | `mtx_in`, `mtx_out` e a variável de condição `cv_out`, que acorda a `T_IO` quando há setpoint novo. | `imagem_es.hpp` |
| Pedido de rota | `mtx_nav` + `cv_rota` com predicado de versão. | `rota.cpp` |
| Criação de processos | Fila local + `cv_spawn` entre `T_GES_ANUNCIO` e `T_GES_SPAWN`. | `gestor.cpp` |
| Parada de emergência | `parada_geral` atômica no cabeçalho, lida sem trava a cada ciclo da `T_NAV`. | `navegacao.cpp` |

**Hierarquia de travas.** Só se adquire uma trava de nível inferior enquanto se segura uma de nível superior: N1 equipe → N2 slot do robô / espelho de passagem → N3 `mtx_modelo`, `mtx_nav` → N4 `mtx_in`, `mtx_out` → N5 buffer de eventos, console. Por isso o organizador decide sob `mtx_modelo` e só depois, já sem ele, escreve nas equipes e nos slots (estrutura `Efeitos`). Nenhuma seção crítica faz E/S de disco, `fork` ou espera sem prazo; a saída de texto vai para uma fila e é escrita por uma thread própria (`console.cpp`).

## 5. Ciclo de vida de uma carga

1. `T_ORG_DESP` coloca a próxima caixa de cada baia no ponto de retirada (PR).
2. O despachante escolhe, da caixa mais antiga para a mais nova, uma que caiba inteira nos robôs livres e designa os mais próximos. Se a mais antiga passa de 30 s sem atendimento, os robôs livres ficam reservados para ela. Caixa maior que a frota operacional é sinalizada como *aguardando frota* e não reserva ninguém.
3. Cada membro recebe `MISSAO_EQUIPE` com seu afastamento em relação à carga e segue até o ponto de acoplamento (`DESLOCANDO`), passando pelas passagens com o protocolo da seção 4.
4. Ao chegar, envia `EM_POSICAO` e espera (`AGUARDANDO_EQUIPE`). Com todos em posição, o organizador define `t_go`.
5. Todos acionam `o_acoplar_e_erguer` em `t ≥ t_go` (`ACOPLANDO`). Com todos atracados, a equipe passa a `TRANSPORTE`.
6. O líder pede ao `T_ROTA` uma rota que só use passagens com capacidade ≥ *k* e a publica no slot da equipe. Todos seguem `waypoint + afastamento` com a barreira por waypoint; o líder adquire as *k* vagas antes de entrar numa passagem.
7. No destino, o líder avisa o organizador, que define um novo `t_go` para o desacoplamento. Com todos soltos, o líder envia `ENTREGUE` e o coletor registra a entrega.

Se um membro apresenta defeito no caminho, ele para no ciclo seguinte e marca a equipe como `ABORTANDO`; o organizador define `t_go`, todos depositam a carga ao mesmo tempo e a caixa volta ao despacho a partir do ponto onde ficou.

## 6. Compilação e execução

### Dependências

- Linux (nativo ou WSL2)
- `g++` com C++17 e `make`
- Python 3, somente para os scripts que analisam os testes de integração

A leitura do cenário usa a biblioteca [nlohmann/json](https://github.com/nlohmann/json) 3.11.3, incluída em `third_party/` (licença MIT). Não há outra dependência externa.

### Compilar

```bash
make            # compila tudo em bin/
make testes     # testes de domínio (T-01)
make tsan       # versão com ThreadSanitizer em bin-tsan/
```

### Executar

Todo o sistema sobe a partir de um único arquivo, sempre com caminhos relativos à raiz do repositório:

```bash
./run.sh --robos 4
```

O `run.sh` inicia o organizador (que cria os objetos de IPC), o coletor e o gestor, e anuncia os N robôs. `Ctrl+C` encerra tudo na ordem certa (gestor e robôs, coletor, organizador) e remove os objetos de IPC.

| Opção | Padrão | Efeito |
|---|---|---|
| `--robos N` | 4 | Robôs anunciados na partida. |
| `--n-max M` | 32 | Capacidade da tabela da frota. Só dimensiona o IPC; não exige recompilar. |
| `--cenario arquivo` | `config/cenario.json` | Mapa, passagens, baias, caixas e parâmetros. |
| `--sem-rt` | — | Não pede `SCHED_FIFO` (equivale a `AMR_SEM_RT=1`). |
| `--bin pasta` | `bin` | Pasta dos executáveis (por exemplo, `bin-tsan`). |

Em outro terminal:

```bash
./bin/amr_monitor                  # tabela da frota, passagens e equipes, a cada 1 s
./bin/anunciar_robo 9              # robô novo com o sistema rodando
./bin/injetar_evento defeito 2 on  # defeito no robô 2
./bin/injetar_evento defeito 2 off
./bin/injetar_evento reconhecer 2  # operador reconhece a falha
```

Os registros de cada execução ficam em `logs/<data-hora>/` (o atalho `logs/ultima` aponta para a mais recente):

| Arquivo | Conteúdo |
|---|---|
| `cargas.csv` | Cada caixa: disponibilizada, designada, pegou na baia, entregou na zona, abortada — com data e hora e robôs envolvidos. |
| `eventos.csv` | Todos os registros: cargas, robôs (missões, passagens, defeitos, bateria), sistema e operador. |
| `frota.csv` | Amostra de 1 s de cada slot: estado, posição, bateria, distância e métricas de tempo real. |

### Prioridades de tempo real

`SCHED_FIFO` exige `CAP_SYS_NICE` ou `ulimit -r` maior que zero. Sem permissão, o sistema avisa uma vez e roda com a política normal. Para usar as prioridades sem `root`, uma opção é `sudo setcap cap_sys_nice+ep bin/amr_robo bin/amr_organizador` (o gestor e o coletor não usam tempo real).

Se uma execução for interrompida de forma abrupta, `./bin/limpar_ipc` remove a memória compartilhada, as filas e os semáforos que tiverem sobrado. O organizador também faz essa limpeza ao iniciar.

## 7. Cenário

`config/cenario.json` descreve um galpão de 30 m × 20 m (células de 1 m) dividido por uma parede, com três passagens estreitas e um corredor principal ao norte:

```
 y
19 . e e e e e e e e e e e e . . . . . . . . . . . . . . . . .   ← corredor principal
18 . e e e e e e e e e e e e . . . . . . . . . . . . . . . . .     (sem limite de robôs)
17 . e e e e e e e e e e e e . . . . . . . . . . . . . . . . .   e = estacionamento
16 . e e e e e e e e e e e e . . . . . . . . . . . . . . . . .       dos robôs anunciados
15 . . . . . . . . . . . . . . # # . . . . . . . . . . . . . .
13 . . . . B3. . . . . . . . . P3P3. . . . . . . . . . Z3. . . .   P3: capacidade 3
 8 . . . . B2. . . . . . . . . P2P2. . . . . . . . . . Z2. . . .   P2: capacidade 2
 3 . . . . B1. . . . . . . . . P1P1. . . . . . . . . . Z1. . . .   P1: capacidade 1
 1 R . . . . . . . . . . . . . # # . . . . . . . . . . . . . R   R = recarga
 0 R . . . . . . . . . . . . . # # . . . . . . . . . . . . . R
   0         5        10        15        20        25       29  x
```

Baias (B) à esquerda, zonas de destino (Z) à direita. Equipes de 4 só passam pelo corredor principal; equipes de 2 ou 3, pelas passagens que comportam o seu tamanho. Os parâmetros (velocidade 1 m/s, limiares de bateria 30 % / 15 % / 95 %, envelhecimento de 30 s, prazos de reunião e acoplamento) também estão no arquivo.

Os arquivos `config/testes_*.json` usam o mesmo galpão com filas maiores de caixas e robôs a 2 m/s, para que os testes de integração terminem mais rápido.

## 8. Ferramentas

| Ferramenta | Uso |
|---|---|
| `anunciar_robo ID [X Y]` / `anunciar_robo --n N [--intervalo-ms T]` | Anuncia robôs ao gestor (na Etapa 2 o anúncio virá do simulador). |
| `injetar_evento defeito ID on\|off`, `bateria ID NIVEL`, `comunicacao ID off\|on` | Injeta eventos de hardware no `DriverStub` do robô. |
| `injetar_evento parada-geral`, `liberar-parada`, `caixa ID TAM BAIA ZONA` | Comandos de operador ao organizador. |
| `injetar_evento parar\|retomar\|reconhecer\|abortar ID`, `modo ID auto\|manual`, `destino ID X Y`, `remover-robo ID` | Comandos de operador encaminhados a um robô. `remover-robo` devolve as vagas de passagem de um robô que o operador retirou do galpão. Passar um robô para `manual` no meio de uma missão aborta a equipe dele e devolve a caixa ao despacho. |
| `amr_monitor [--uma-vez] [--csv] [--intervalo-ms T]` | Lê a tabela da frota (sem alterá-la) e mostra robôs, passagens e equipes. |
| `limpar_ipc` | Remove objetos de IPC remanescentes. |
| `rajada_comandos ID N` | Usada no teste T-04: N missões com um `PARAR` no meio, de uma vez na fila do robô. |

## 9. Testes

Os testes seguem o plano do relatório da Etapa 1. Cada script grava as evidências em `testes/resultados/T-XX/` (log da execução, CSVs do coletor, plataforma e resultado).

| ID | Script | O que verifica |
|---|---|---|
| T-01 | `make testes` ou `t01_dominio.sh` | Domínio isolado: A*, seguidor de waypoints, bateria, borda de defeito, formação e despachante. |
| T-02 | `t02_tsan.sh` | Organizador, coletor, gestor e 4 robôs com ThreadSanitizer por 10 min (`DURACAO`). |
| T-03 | `t03_status.sh` | Cada leitura do `amr_monitor` é idêntica ao que a `T_STATUS` publicou com o mesmo *heartbeat* (sem leitura rasgada), com N = 8. |
| T-04 | `t04_comandos.sh` | `PARAR` ultrapassa as missões pendentes na fila do robô; nada perdido ou duplicado. |
| T-05 | `t05_criacao.sh` | Um processo por robô até N_max, um anúncio a mais recusado e `kill -9` detectado. |
| T-06 | `t06_passagens.sh` | Seis robôs e equipes de 2 e 3 por 10 min; ocupação de cada passagem nunca acima da capacidade. |
| T-07 | `t07_acoplamento.sh` | Dispersão entre os acionamentos de acoplamento de 50 equipes (limite de 60 ms) e nenhum antes de `t_go`. |
| T-08 | `t08_despacho.sh` | Caixa de tamanho 4 entregue em até 3 min entre caixas de tamanho 1; caixa maior que a frota sinalizada sem travar o resto. |
| T-09 | `t09_mutex_robusto.sh` | 20 robôs morrem segurando o mutex do slot; o organizador recebe `EOWNERDEAD` e segue. |
| T-10 | `t10_coletor.sh` | Coletor pausado com `SIGSTOP` por 30 s: robôs não bloqueiam, registros chegam depois com data e hora. |
| T-11 | `t11_defeito.sh` | 100 defeitos injetados em robôs em movimento: setpoint congelado em até dois ciclos da `T_NAV` após a detecção. |
| T-12 | `t12_frota_n.sh` | T-03 repetido com N = 1, 2, 4, 8 e 16; com N = 1 só caixas de tamanho 1 são designadas. |

```bash
bash testes/integracao/t07_acoplamento.sh                 # um teste
EQUIPES=10 bash testes/integracao/t07_acoplamento.sh      # versão curta
bash testes/integracao/executar_todos.sh                  # todos, em sequência (cerca de 1 h)
```

O teste T-09 usa um gancho de teste: com `AMR_TESTE_MORRER_NO_SLOT="CICLO@ID_MIN"`, robôs com id ≥ `ID_MIN` se matam com `SIGKILL` dentro da seção crítica do slot no ciclo indicado. Com `AMR_RASTREAR_STATUS=1`, cada robô registra no log tudo o que publica no slot (usado no T-03).

## 10. Organização do código

```
Makefile               alvos make, make testes, make tsan, make clean
run.sh                 lança e encerra o sistema
config/                cenário padrão e cenários dos testes
src/comum/             biblioteca comum (libamr_comum.a)
  ipc.*                RAII de shm, filas, semáforos POSIX e System V, mutex compartilhado robusto
  frota.*              leiaute de /amr_frota
  protocolo.*          catálogo de mensagens (128 bytes)
  eventos.*            buffer circular /amr_eventos e produtor (Rastreador)
  tarefa_periodica.*   tarefa periódica com liberação absoluta e métricas
  cenario.*, mapa.*    leitura do cenário e planejador A*
  console.*            saída de texto assíncrona
src/robo/              amr_robo: navegacao (T_NAV), status (T_STATUS), rota (T_ROTA),
                       driver_stub (T_IO), dominio.hpp, imagem_es.hpp, nucleo.*
src/organizador/       amr_organizador: organizador (3 threads), despachante (domínio), modelo
src/gestor/            amr_gestor
src/coletor/           amr_coletor
ferramentas/           anunciar_robo, injetar_evento, amr_monitor, limpar_ipc, rajada_comandos
testes/                testes de domínio (teste_dominio.cpp) e de integração (integracao/)
third_party/nlohmann/  json.hpp
```

## 11. Limitações conhecidas e próximos passos

- Não há detecção de colisão entre robôs: o sistema anticolisão descentralizado é um dos pontos extras previstos para a Etapa 2.
- O `DriverStub` é uma cinemática simplificada (ponto material com velocidade e aceleração limitadas); a física e a geração de eventos passam ao simulador na Etapa 2, por trás da mesma interface `DriverES`.
- A rota alternativa após espera longa numa passagem é pedida por robôs sozinhos; uma equipe parada na entrada de uma passagem gera alarme na supervisão, mas continua esperando a vaga.
- Posições e afastamentos são trocados em metros (`float`), e não em decímetros como no catálogo de mensagens do relatório.
- Etapa 2: `DriverMQTT`, anúncio de robôs e comandos via MQTT, `T_ORG_HMI`, `T_COL_PUB`, simulador em Node.js, tela de operação no navegador com o galpão em 3D, e a análise de quantos robôs o hardware suporta. Esse trabalho está no repositório [projeto-final-atr](https://github.com/felipesvm/projeto-final-atr).
