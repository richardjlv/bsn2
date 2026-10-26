# Mudanças no código-fonte do ROS2 durante a validação da migração

Este arquivo registra **toda alteração feita em código de produção do
`bsn_ros2`** (não em testes) durante a validação da migração ROS1 → ROS2
contra as suítes BDD. Para cada uma: o que mudou, o sintoma que motivou a
mudança, a referência no ROS1 que define o comportamento correto, e por que
não é uma regressão.

**Escopo.** Só código de produção, nos pacotes `adaptation/`, `central_hub/`
e `system_monitor/`. As mudanças do lado dos testes (`features/`) estão
listadas em [Apêndice: mudanças do lado dos testes](#apêndice-mudanças-do-lado-dos-testes),
sem detalhamento — elas não alteram o comportamento do sistema. Gaps que
continuam **abertos** (nada foi implementado) estão em
[`MANAGING_SYSTEM_GAPS.md`](MANAGING_SYSTEM_GAPS.md),
[`DATA_PERSISTANCE_GAPS.md`](DATA_PERSISTANCE_GAPS.md) e
[`INJECTOR_GAPS.md`](INJECTOR_GAPS.md); ver
[O que ficou de fora](#o-que-ficou-de-fora-gaps-ainda-abertos).

## Resumo

Quatro classes de mudança, por natureza do problema:

- 🟥 **Caminho morto** — código que nunca executou (exceção não tratada,
  nome de tópico que não casa, caminho de arquivo inexistente, nó nem
  iniciado). Não havia comportamento a preservar: o ROS1 executava esse
  caminho, o porte não.
- 🟨 **Config divergente** — valor de parâmetro diferente do launch de
  produção do ROS1 (`bsn_full.launch`).
- 🟦 **Algoritmo divergente** — lógica que não reproduzia a implementação
  C++ de referência.
- ⬜ **Topologia** — mudança de roteamento. Houve uma (`log_adapt`/`reconfigure`),
  **já revertida** para a topologia do ROS1. Ver seção 4.

| Arquivo | | Mudança |
|---|---|---|
| [`adaptation/data_access/data_access.py`](../adaptation/adaptation/data_access/data_access.py) | 🟥 | `DataAccess()` → `DataAccess(node_name="data_access")`; tópico `TargetSystemData` → `target_system_data`; `models_path` via `get_package_share_directory` |
| [`adaptation/setup.py`](../adaptation/setup.py) | 🟥 | instala `resource/model/*` no share (pré-requisito do item acima) |
| [`adaptation/engines/base_engine.py`](../adaptation/adaptation/engines/base_engine.py) | 🟥 | `future.result()` → `future.result().content`; esperas com timeout; `rate.sleep()` → `time.sleep(period)` |
| [`adaptation/engines/reli_engine.py`](../adaptation/adaptation/engines/reli_engine.py) | 🟥 🟦 | `.content` nas duas respostas; `plan()` reescrito como busca por coordenadas |
| [`adaptation/enactor/enactor.py`](../adaptation/adaptation/enactor/enactor.py) | 🟦 | `NODE_TO_BSN_COMPONENT` / `bsn_component_name`; único publisher de `AdaptationCommand` é `log_adapt` (como no ROS1) |
| [`adaptation/enactor/controller.py`](../adaptation/adaptation/enactor/controller.py) | 🟥 🟦 | tradução de nome em `receive_event`; guarda de re-inicialização no `"activate"`; comando publicado só em `log_adapt`; `_publish_exception` no formato `<componente>=<valor>` |
| [`adaptation/config/enactor.yaml`](../adaptation/config/enactor.yaml) | 🟨 | `kp: 150.0` |
| [`adaptation/config/reli_engine.yaml`](../adaptation/config/reli_engine.yaml) | 🟥 🟨 | chave do nó `reliability_engine` → `reli_engine`; `setpoint`/`offset`/`gain`/`actuation_freq`/`info_quant` alinhados ao ROS1 |
| [`central_hub/config/central_hub.yaml`](../central_hub/config/central_hub.yaml) | 🟨 | `frequency: 2.0` → `6.0` |
| [`central_hub/launch/emergency_detection_launch.py`](../central_hub/launch/emergency_detection_launch.py) | 🟥 | nó `enactor` descomentado |
| [`system_monitor/param_adapter.py`](../system_monitor/system_monitor/param_adapter.py) | 🟥 | normalização de barra no lookup; `NameError` em `target` |
| [`system_monitor/node_monitor.py`](../system_monitor/system_monitor/node_monitor.py) | 🟥 | `check_state_helper` não chama mais `call_async(GetState)` |
| [`system_monitor/logger.py`](../system_monitor/system_monitor/logger.py) | — | relay `log_adapt` → `reconfigure` removido em `fde82d8` e **restaurado** (seção 4) |
| [`central_hub/launch/emergency_detection_adaptation_launch.py`](../central_hub/launch/emergency_detection_adaptation_launch.py) (novo) | — | variante de launch para um cenário do BSN-P03 |

## 1. Caminho morto: código que nunca executou

Todos os itens desta seção têm a mesma justificativa de não-regressão, então
ela vale uma vez para a seção inteira: **não havia comportamento anterior a
preservar**. Cada um desses caminhos abortava com exceção, nunca casava, ou
nunca era iniciado — enquanto o equivalente no ROS1 executava normalmente.
Consertá-los só pode aproximar o porte do ROS1, não afastá-lo.

### 1.1 `data_access` não subia — `TypeError` no `main()`

`DataAccess.__init__` exige `node_name` como parâmetro posicional
([`data_access.py:116`](../adaptation/adaptation/data_access/data_access.py#L116)),
e o `main()` instanciava `DataAccess()` sem argumento nenhum: o nó levantava
`TypeError` antes de existir. Todo cenário que depende do repositório de
conhecimento (`knowledge_repo.feature`, e as consultas de confiabilidade de
`managing_system.feature`) dependia de um nó que nunca chegou a subir.

Passamos `node_name="data_access"` — o mesmo nome que o launch, que
`param_adapter.bsn_to_ros_mapping` e que `utils/interface_map.py` já
esperavam.

### 1.2 `data_access` nunca recebia `TargetSystemData`

O subscriber usava `"TargetSystemData"` — o nome da *classe da mensagem* —
enquanto o publisher do hub central anuncia `"target_system_data"`
([`publishers.py:68`](../central_hub/central_hub/components/publishers.py#L68)).
Nomes de tópico no ROS2 são literais e sensíveis a caixa, então as duas
pontas nunca se encontraram.

Nuance importante: no ROS1 o tópico **realmente** se chamava
`TargetSystemData` ([`DataAccess.cpp:102`](../../bsn2/src/sa-bsn/knowledge_repository/data_access/src/DataAccess.cpp#L102)).
A renomeação para snake_case foi uma decisão prévia do porte, aplicada só no
publisher. A correção alinha o subscriber ao publisher deste porte — não ao
nome literal do ROS1 —, porque é o publisher que define o tópico que existe
em tempo de execução.

### 1.3 Fórmulas e goal model nunca eram carregados

`models_path` era `"src/adaptation/resource/model"`: um caminho relativo ao
diretório de trabalho do processo, que não corresponde a este layout (o
pacote vive em `src/bsn_ros2/adaptation`). Toda busca de fórmula e de goal
model falhava silenciosamente.

Agora resolve pelo share do pacote instalado
(`get_package_share_directory("adaptation")`), e o
[`setup.py`](../adaptation/setup.py) passou a instalar `resource/model/*` —
as duas metades são necessárias juntas.

### 1.4 Engines tratavam o objeto de resposta como string

`future.result()` de um `DataAccessRequest` devolve uma mensagem de resposta
com um único campo `string content`, não a string. Todo `.strip()`/`.split()`
subsequente operava sobre um `DataAccessRequest_Response`. Três ocorrências:
[`base_engine.py`](../adaptation/adaptation/engines/base_engine.py) (fórmula),
[`reli_engine.py:115`](../adaptation/adaptation/engines/reli_engine.py#L115)
(confiabilidade) e
[`reli_engine.py:183`](../adaptation/adaptation/engines/reli_engine.py#L183)
(contexto). Todas passaram a ler `.content`.

### 1.5 `Engine.body()` parava depois de uma iteração

O laço usava `self.create_rate(...).sleep()`. Um `Rate` do rclpy é movido por
um timer, que só dispara enquanto **algum executor gira o nó**; o `main()`
roda esse laço na única thread que existe. Sem ninguém girando o nó em
paralelo, o primeiro `sleep()` bloqueia para sempre: o laço logava
`"Starting engine body loop..."`, fazia uma passada de monitoramento e nunca
mais iterava — nenhuma `strategy` era publicada.

Trocado por `time.sleep(period)`, mantendo o `rclpy.spin_once(...)` que o
próprio laço já fazia para bombear callbacks.

### 1.6 `Engine.get_formula()` podia travar antes de o nó terminar de subir

`setup()` chama esse método de dentro de um laço `while not formula_str`,
antes de o executor girar. Uma requisição enviada antes de o DDS casar com o
serviço do `DataAccess` é simplesmente descartada, e o future nunca completa
— com espera ilimitada, o nó travava antes de criar seu publisher
`strategy`. Agora as duas esperas são limitadas
(`wait_for_service(timeout_sec=5.0)` e
`spin_until_future_complete(..., timeout_sec=5.0)`), devolvendo `""` para o
laço de retry que já existia.

### 1.7 `param_adapter` parava de rotear na primeira mensagem desconhecida

Dois defeitos no mesmo handler
([`param_adapter.py:193-225`](../system_monitor/system_monitor/param_adapter.py#L193-L225)):

1. O ramo de "alvo desconhecido" referenciava uma variável `target` que
   nunca foi definida. O `NameError` escapava do callback da subscription e
   desmontava a thread de spin do nó — **um único comando não roteável
   parava toda a reconfiguração pelo resto da vida do processo.**
2. `bsn_to_ros_mapping` e `_component_publishers` são chaveados com barra
   inicial ([`param_adapter.py:76-86`](../system_monitor/system_monitor/param_adapter.py#L76-L86)),
   mas um componente que se identifica por `node.get_name()` não tem barra —
   então um alvo perfeitamente válido errava as duas tabelas e caía
   justamente no ramo defeituoso acima.

### 1.8 `node_monitor.check_heartbeats` desmontava a thread de spin

`check_state_helper` fazia `self.state_clients[node].call_async(GetState)`,
passando a *classe* do serviço onde o rclpy espera uma instância de request:
`TypeError` dentro do callback do timer, desmontando o spin e, com ele,
parando silenciosamente o relay `collect_event` → `log_event`. Além disso o
retorno era um `Future`, que nunca poderia ser igual à constante de estado
comparada em
[`node_monitor.py:296`](../system_monitor/system_monitor/node_monitor.py#L296);
e esperar a resposta ali dentro daria deadlock, porque o timer roda no
executor do próprio nó.

Agora o helper responde com o último estado conhecido que o monitor **já
rastreia** (`monitored_nodes[node]['active']`, atualizado pelos eventos que
ele recebe) — sem chamada de serviço, sem espera.

### 1.9 O `enactor` não era nem iniciado

O nó estava inteiro comentado no
[`emergency_detection_launch.py`](../central_hub/launch/emergency_detection_launch.py#L145-L158).
Sem ele não existe laço de adaptação nenhum: nada consome `strategy`, nada
publica `reconfigure`. Descomentado, com `parameters=[config_enactor] if
os.path.exists(config_enactor) else []` para não quebrar quando o yaml não
está instalado.

## 2. Config divergente do launch de produção do ROS1

Todos os valores abaixo foram conferidos contra os launches que o
[`bsn_full.launch`](../../bsn2/bsn_full.launch) de fato inclui — não contra
os `test_*.launch`.

| Parâmetro | Antes (ROS2) | Agora | Referência no ROS1 |
|---|---|---|---|
| `central_hub.frequency` | `2.0` | `6.0` | [`g4t1.launch`](../../bsn2/src/sa-bsn/configurations/target_system/g4t1.launch) (`value="6"`), incluído por `bsn_full.launch:12` |
| `enactor.kp` | ausente → default `0.5` do código | `150.0` | [`strategy_enactor.launch`](../../bsn2/src/sa-bsn/configurations/system_manager/strategy_enactor.launch) (`value="150"`) |
| `reli_engine.setpoint` | `0.95` | `0.9` | [`strategy_manager.launch`](../../bsn2/src/sa-bsn/configurations/system_manager/strategy_manager.launch) |
| `reli_engine.offset` | `0.05` | `0.5` | idem |
| `reli_engine.gain` | `0.2` | `0.01` | idem |
| `reli_engine.actuation_freq` | `0.5` | `0.02` | idem |
| `reli_engine.info_quant` | `5` (int) | `5.0` | idem (`value="5"`); float porque o parâmetro é declarado como double em [`base_engine.py:44`](../adaptation/adaptation/engines/base_engine.py#L44) — um int aqui não aplica |
| `reli_engine.monitor_freq` | `1.0` | `1.0` (inalterado) | idem |
| `reli_engine.tolerance` | `0.02` | `0.02` (inalterado) | default no construtor, [`ReliabilityEngine.cpp:16`](../../bsn2/src/sa-bsn/system_manager/adaptation_engine/src/ReliabilityEngine.cpp#L16) |

Duas observações que valem mais que a tabela:

- **A chave do nó no `reli_engine.yaml` estava errada** (`reliability_engine:`
  em vez de `reli_engine:`, o nome real do nó no launch). Com a chave errada,
  o ROS2 ignora o arquivo **inteiro** e todo parâmetro cai no default do
  código — por isso essa linha conta como 🟥, não só 🟨: nenhum dos valores
  acima estava em vigor, nem os que já estavam corretos.
- **`kp` nunca tinha sido setado**, então o `Controller` usava seu default
  Python de `0.5` — um ganho proporcional 300× menor que o do ROS1. Como
  `new_freq = freq + (kp/100) * error` e os termos de erro ficam bem abaixo
  de 1.0, isso movia a frequência em centésimos de Hz por ciclo:
  tecnicamente convergente, mas lento demais para cruzar um valor `freq=`
  de 2 casas decimais dentro de qualquer janela de teste ou operação.

**Por que não é regressão:** cada valor passou a ser o do ROS1, verificado
arquivo por arquivo. O que muda de comportamento é justamente o que estava
divergindo do sistema de referência.

## 3. Algoritmo divergente da implementação C++

### 3.1 `reli_engine.plan()` — reset dos termos e busca por coordenadas

Duas divergências em relação a
[`ReliabilityEngine.cpp`](../../bsn2/src/sa-bsn/system_manager/adaptation_engine/src/ReliabilityEngine.cpp):

1. **Reset dos termos `R_`.** A versão anterior amarrava o reset ao *sinal do
   erro* (`r_curr` se `error > 0`, senão `1.0` para todo termo ativo), o que
   empurrava todo componente para o teto sempre que o sistema já estava
   sobre-confiável. O C++ (linhas 203-216) amarra ao *estado do componente*:
   `r_curr` para um componente ativo e não desativado, `1.0` para um
   desativado. É o que fazemos agora — e não era necessário forçar o teto,
   porque a busca abaixo já decide a direção de cada termo.
2. **Geração de soluções.** A versão anterior dava **um único passo**
   `gain * error` em **um único termo** por solução. A fórmula deste sistema
   multiplica sete componentes
   ([`reliability.formula`](../adaptation/resource/model/reliability.formula)),
   então com mais de um componente degradado um passo pequeno num termo só
   nunca leva o agregado para dentro de uma banda de ±2%. Agora é a busca
   por coordenadas do C++ (linhas 224-283): para cada termo candidato,
   desloca todos os termos ativos em direção a `r_curr`, sobe o candidato
   passo a passo até o setpoint, e então sobe cada um dos outros termos do
   mesmo jeito, cada etapa fixando o resultado das anteriores.

**Sintoma observado antes:** `plan()` logava `"Did not converge :("` a cada
ciclo, com o erro medido crescendo em vez de diminuir.

**Por que não é regressão:** a estrutura agora acompanha a implementação de
referência passo a passo; o comportamento anterior não correspondia a nenhuma
versão do sistema.

### 3.2 Espaço de nomes BSN vs. nome de nó ROS2

Todo o subsistema de adaptação fala o espaço de nomes BSN de ponta a ponta:
`DataAccess` reporta confiabilidade por `"/g3t1_1"`, o `ReliabilityEngine`
emite estratégias para esses mesmos nomes, e
`Controller.apply_reli_strategy` já tratava `"/g4t1"` como caso especial do
hub. Os `Event`s de ciclo de vida são a exceção: carregam
`node.get_name()` (`"oximeter_node"`). Sem tradução na chegada, um componente
terminava com `r_curr`/`r_ref` chaveados por nome BSN e
`freq`/`exception_buffer`/`kp_individual` chaveados por nome de nó — e
`apply_reli_strategy` levantava `KeyError` no primeiro deles que tocasse.

Adicionamos `NODE_TO_BSN_COMPONENT` e `bsn_component_name()` (idempotente)
em [`enactor.py`](../adaptation/adaptation/enactor/enactor.py), aplicados em
`Controller.receive_event`. O mapa é mantido em sincronia com os dois que já
existiam: `data_access.component_mapping` e
`param_adapter.bsn_to_ros_mapping`.

**Por que não é regressão:** no ROS1 esse problema não existe porque o nome
do componente é a mesma string em todos os tópicos (`/g3t1_1`). A tradução é
um artefato da renomeação de nós feita pelo porte, e resolve uma
inconsistência interna dele — não altera nenhum comportamento observável do
ROS1.

### 3.3 `"activate"` como heartbeat — guarda de re-inicialização

Esta é a única mudança da seção 3 que **se afasta deliberadamente da letra**
do C++, e vale registrar o porquê com precisão.

- No ROS1, `"activate"` é um evento de **transição**: `Component::activate()`
  faz `sendEvent("activate")` uma vez e marca `status = true`
  ([`Component.cpp:152-155`](../../bsn2/src/archlib/src/target_system/Component.cpp#L152-L155)).
  Por isso `Controller::receiveEvent` pode re-inicializar
  `invocations`/`r_curr`/`r_ref`/`kp`/`freq`/`exception_buffer`
  incondicionalmente a cada `"activate"`
  ([`Controller.cpp:39-53`](../../bsn2/src/sa-bsn/system_manager/enactor/src/Controller.cpp#L39-L53)).
- Neste porte, `"activate"` virou **heartbeat**: o sensor publica um a cada
  ciclo do laço de coleta
  ([`sensor.py:110`](../sensor/sensor/sensor.py#L110)) e o `node_monitor`
  repassa todos eles (os eventos servem de sinal de vivacidade — ver a
  docstring de `event_callback`). São centenas por componente.

O porte fiel da re-inicialização incondicional, somado a essa emissão
contínua, resetava `freq`/`r_curr`/`r_ref` para os defaults várias vezes por
segundo — descartando tudo que o laço de controle havia acumulado e fixando
a frequência comandada no valor inicial. A guarda (`if component in
self.freq: return`) restaura a **intenção** do código do ROS1 —
inicializar uma vez por ativação — sob o padrão de emissão diferente deste
porte. Uma reativação genuína ainda inicializa, porque o ramo `"deactivate"`
remove essas chaves.

## 4. `log_adapt` / `reconfigure`: topologia do ROS1 restaurada

**No ROS1**, o `Controller` anuncia um único publisher de
`AdaptationCommand`, e ele é o `log_adapt`
([`Controller.cpp:12`](../../bsn2/src/sa-bsn/system_manager/enactor/src/Controller.cpp#L12)).
O `Logger` assina `log_adapt`, persiste em `persist` **e** repassa para
`reconfigure`
([`Logger.cpp:15-16,39-47`](../../bsn2/src/sa-bsn/logging_infrastructure/logger/src/Logger.cpp#L39-L47)).
A entrega do comando passa pelo logger, e é isso que o cenário
*The system log sends reconfiguration requests to the parameter adapter*
(Gherkin idêntico nas duas versões) descreve.

**No ROS2 original (antes desta validação)**, o enactor publicava direto em
`reconfigure` e ninguém publicava em `log_adapt`. O relay do logger nunca
disparava, e nenhum `AdaptationCommand` chegava a `/persist`.

**Primeira correção (commit `fde82d8`), depois revertida:** o enactor passou
a publicar nos dois tópicos, e o relay do logger foi removido para evitar
entrega dupla. Fechava o gap de persistência, mas mantinha a topologia
divergente. O cenário acima só passava porque o `reconfigure_pub` do logger
continuava declarado, sem nunca publicar.

**Estado atual (não commitado):** a topologia do ROS1 foi restaurada. O único
publisher de `AdaptationCommand` do enactor é `log_adapt`
([`enactor.py`](../adaptation/adaptation/enactor/enactor.py),
[`controller.py::_send_adaptation_command`](../adaptation/adaptation/enactor/controller.py)),
e o logger persiste e repassa para `reconfigure`
([`logger.py::receive_adaptation_command`](../system_monitor/system_monitor/logger.py)).
Cada comando é entregue uma única vez, e o alvo persistido é o mesmo alvo
entregue (forma BSN, traduzida pelo `param_adapter`). A entrega volta a
depender do logger estar vivo, exatamente como no ROS1.

## 5. Arquivo novo: `emergency_detection_adaptation_launch.py`

Variante de launch para os cenários BDD que precisam do canal de adaptação
de um sensor realmente ligado (`BSN-P03.feature`, cenário "...at its maximum
sampling rate"). Duas diferenças em relação ao
`emergency_detection_launch.py`, ambas necessárias juntas: o
`thermometer_node` recebe `enable_adaptation: true` (para que
`Sensor.__init__` chame `register_with_effector()`), e um `param_adapter` é
iniciado (para que exista o serviço `EffectorRegister` com o qual registrar).
Não altera o launch de produção — é um arquivo separado. O raciocínio
completo está na docstring do próprio arquivo.

## O que ficou de fora (gaps ainda abertos)

Nada foi implementado para estes; ficam registrados aqui só para o
inventário estar completo:

| Gap | Estado | Onde está documentado |
|---|---|---|
| Nó `/injector` (injeção de incerteza) | **Não portado.** Não existe em nenhum pacote do `bsn_ros2`; o `logger` já assina `log_uncertainty`, mas nada publica lá. Os 2 cenários estão `xfail`. | [`INJECTOR_GAPS.md`](INJECTOR_GAPS.md) |
| Registro de falha de persistência (`PersistenceFailure`) | **Nunca existiu em nenhuma das duas versões.** Cenário `xfail` nos dois lados, com a asserção correta no lugar. | [`DATA_PERSISTANCE_GAPS.md`](DATA_PERSISTANCE_GAPS.md) |
| `Controller._publish_exception` no formato `<componente>=<valor>` | **Aplicada (não commitada) e validada ao vivo em 2026-09-19: o canal de exceções funciona, mas o cenário *sampling rate decreases after sensor reliability recovers* continua `xfail`.** Não era a causa (ou não a causa completa); a causa restante é desconhecida. | [`MANAGING_SYSTEM_GAPS.md`](MANAGING_SYSTEM_GAPS.md) |

## Evidência e o que falta reverificar

Cada mudança de valor ou de algoritmo acima tem uma referência verificável
no código/launch do ROS1, listada na própria seção — essa é a base do
argumento de "não é regressão", e ela é estática: não depende de execução.

Do lado dinâmico, o estado validado ao vivo em **2026-09-19**, com o código
atual (incluindo a topologia do ROS1 restaurada na seção 4 e o
`_publish_exception` corrigido), rodando cada suíte `test_*_bdd.py` uma por
vez e lendo o resumo do pytest **interno**:

| Suíte | Resultado |
|---|---|
| `managing_system` | 9 passed, 2 xfailed |
| `target_system` | 18 passed |
| `knowledge_repo` | 1 passed |
| `data_persistance` | 1 passed, 1 xfailed |
| `check_bsn` | 8 passed |
| `health_status` | 2 passed |
| `injector` | 7 xfailed (2 cenários; o Outline expande em 6 exemplos) |
| `bsn_p03` | 2 passed |

Nenhuma falha, nenhum erro e nenhum `XPASS`. Detalhes por feature em
[`MANAGING_SYSTEM_GAPS.md`](MANAGING_SYSTEM_GAPS.md),
[`DATA_PERSISTANCE_GAPS.md`](DATA_PERSISTANCE_GAPS.md) e
[`INJECTOR_GAPS.md`](INJECTOR_GAPS.md).

Sobre `managing_system`: a contagem (9 passed, 2 xfailed) é a mesma de antes
da restauração do relay `log_adapt` → `reconfigure`, então a reversão da seção
4 não causou regressão nesses cenários. O `xfail` de *Sampling rate decreases…*
**persiste com o `_publish_exception` corrigido**; ver o estado detalhado em
`MANAGING_SYSTEM_GAPS.md`.

**Como as suítes foram invocadas** (e uma correção). Cada arquivo
`features/test/test_<feature>_bdd.py` roda com `python3 -m pytest` a partir de
`features/test/`; o resultado que importa é a linha de resumo do pytest
*interno* (o `1 passed` externo só confirma que o harness terminou com código
0, e um `xfail` também dá 0). Uma versão anterior desta seção afirmava que
rodar via `pytest` "derrubava a pilha inteira antes de qualquer step BDD
executar". **Isso estava errado:** eu havia lido só o final do log, que
mostra o desligamento normal por SIGINT (nós saindo com `KeyboardInterrupt`) e
o passo vazio pós-desligamento do `launch_testing` (`Ran 0 tests`, normal).
Duas armadilhas reais, essas sim:

- O `install/` é cópia (não `--symlink-install`): depois de editar o código-fonte
  é preciso recompilar
  (`colcon build --base-paths src/bsn_ros2 --packages-select …`; um `colcon build`
  simples falha por nomes de pacote duplicados com `src/bsn2_test`), senão a
  suíte testa código antigo.
- Duas execuções no mesmo host e domínio DDS se contaminam mutuamente. Em
  2026-09-19 isso ocorreu nos dois sentidos: uma stack de diagnóstico minha
  fez falharem 6 testes unitários da outra sessão, e testes unitários dela
  vazaram nós `core_cov_sensor_*` para o grafo da minha rodada. As 8 suítes
  acima foram rodadas sem outra execução concorrente. Em máquina compartilhada,
  usar um `ROS_DOMAIN_ID` próprio por execução.

## Apêndice: mudanças do lado dos testes

Listadas só para delimitar o escopo — não alteram comportamento do sistema:

```
features/data_persistance.feature          features/steps/test_data_persistance.py
features/knowledge_repo.feature            features/steps/test_knowledge_repo.py
features/managing_system.feature           features/steps/test_managing_system.py
features/package.xml                       features/steps/utils/constants.py
features/test/test_knowledge_repo_bdd.py   features/steps/utils/interface_map.py
features/test/test_managing_system_bdd.py
```
