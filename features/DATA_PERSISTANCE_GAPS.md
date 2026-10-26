# data_persistance.feature — análise de gaps (migração ROS1 → ROS2)

**Status atual, validado ao vivo contra uma pilha real
(`emergency_detection_launch.py`):**

| Cenário | Tag | Status |
|---|---|---|
| A vital sign reading collected by a sensor is persisted in the knowledge repository (BSN-P08) | `@behavior @bsn-p08 @persistence` | 🟢 passa de verdade |
| A persistence failure is recorded when the knowledge repository cannot store a reading (BSN-P08) | `@gap` | 🔴 `xfail` — gap real nas duas versões: nunca foi implementado, nem no ROS1 nem no ROS2 (não é regressão) |

Um achado extra, tangencial às duas cenas mas registrado numa versão
anterior deste arquivo como "defeito latente" — a assinatura de energia do
`node_monitor` —, **também foi confirmado corrigido** ao vivo. Ver abaixo.

## Cenário 1 (caminho feliz): passa

`features/steps/test_data_persistance.py`'s
`reading_retrievable_from_knowledge_repository` checa o que o teste do ROS1
de fato verificava — um registro produzido pelo sensor chegando em
`/persist` —, não a cláusula literal do Gherkin ("with the value reported").
Nenhuma das duas versões jamais guardou o valor bruto da leitura (ver
"O que nenhuma versão faz", abaixo); pedir isso seria escopo novo, não uma
regressão.

Confirmado ao vivo: capturando `/persist` durante ~34s de execução da
suíte, chegam registros contínuos com `source="thermometer_node"` e um
`type` não vazio, enquanto o termômetro publica. O teste passa
(`1 passed`).

## Achado resolvido: a assinatura de energia do `node_monitor`

Uma versão anterior deste arquivo documentava que `node_monitor.py`
assinava o tópico plano `/collect_energy_status`, enquanto cada sensor (e o
hub) publicam em `/collect_energy_status/<componente>` — logo nada nunca
chegava, e `/persist` nunca carregava um registro `EnergyStatus`.

**Isso não é mais verdade.** `node_monitor.py` hoje itera sobre cada nó
monitorado e assina `collect_energy_status/<nome>` individualmente
([`node_monitor.py:123-135`](../system_monitor/system_monitor/node_monitor.py#L123-L135)).
Confirmado ao vivo, dois jeitos independentes:

```
$ ros2 topic echo /log_energy_status
source: central_hub_node
content: energy:499.94:cost:0.01
---
source: thermometer_node
content: energy:85.82:cost:2.50
```

```
$ ros2 topic echo /persist --field type   # 8s de captura
    195 Status
     83 EnergyStatus
     28 AdaptationCommand
     24 Event
```

O canal está completo hoje, ponta a ponta:
`collect_energy_status/<componente>` → `node_monitor` → `log_energy_status`
→ `logger` → `/persist`, com dados reais fluindo. Nenhuma cena deste arquivo
dependia disso diretamente, mas como estava documentado aqui como um gap
aberto, o registro merece constar como fechado.

## Cenário 2 (caminho triste): bloqueado nas duas versões — não é regressão

Passou de `skip` para `xfail` em ambos os lados (**2026-09-12**). A teoria
anterior — de que o cenário era autocontraditório como escrito porque o
`Given`/`When` do ROS1 matava o único publicador de `/persist` antes do
`Then` ler o tópico — foi **derrubada por um replay ao vivo** da suíte ROS1
(`rostest ... test_data_persistance.launch --text`, log completo capturado
com `rostopic echo /persist` em paralelo). O que o replay mostrou:

1. **`rosnode.kill_nodes('/logger')` nunca mata o node de fato.** No log do
   rostest, `/logger` aparece exatamente duas vezes: quando sobe
   (`process[logger-7]: started with pid [...]`) e quando o rostest derruba
   tudo no teardown final (`[logger-7] killing on exit`) — nenhuma morte no
   meio do teste. `/persist` nunca fica mudo.
2. Mesmo assim, a asserção antiga (`'fail' in parse_topic_data('/persist')['content']`,
   uma checagem de pertencimento numa tupla de strings) reportava
   **"2 passed"** — mas por um motivo completamente não relacionado:
   `source: "/g4t1"` (o hub central) sozinho respondia por **106523 dos
   121246 registros** capturados em `/persist` (~88%), todos
   `type: "Status", content: "fail"`. É o catch-all de saúde de componente
   do `CentralHub.cpp` (`catch (...) { sendStatus("fail"); }`), que começa a
   disparar sem parar a partir de ~12% da execução — um bug pré-existente,
   não relacionado a persistência, e que nunca vem do termômetro (0 dos
   registros de falha tinham `source` do sensor exercitado pelo cenário).
   `DataAccess::calculateComponentReliability` consome esse mesmo `"fail"`
   para uma métrica de confiabilidade por componente — outro uso legítimo,
   também não relacionado.
3. Não existe, em **nenhum lugar do código-fonte de nenhuma das duas
   versões**, um branch que module "o repositório de conhecimento falhou ao
   persistir esta leitura". A busca pela string `"fail"` no ROS1 encontra só
   esses dois usos (saúde de componente e confiabilidade) — nunca uma falha
   de armazenamento. Ou seja: não é que a feature foi portada pelo caminho
   errado — ela nunca existiu, em nenhuma versão, para ser portada.

Por isso agora é `xfail`, não `skip`: os dois lados rodam a checagem
*correta* (um registro `type == "PersistenceFailure"` identificando a
leitura que falhou) em vez do cheque frouxo que passava por acidente — e
essa checagem falha hoje pelo motivo certo, documentado no próprio teste.
`xfail` também significa que, se algum dia alguém implementar o mecanismo de
verdade, o teste passa a reportar `XPASS`, sinalizando que é hora de remover
a marcação — o que `skip` nunca teria feito.

**No ROS2**, confirmado ao vivo (antes deste replay), dois gaps
independentes em
[`system_monitor/logger.py`](../system_monitor/system_monitor/logger.py),
nenhum deles corrigido desde a última verificação:

1. **Não existe um parâmetro `storage_failure_simulated`.**
   `ros2 param list /logger` mostra só `frequency` (mais os dois padrões de
   framework, `use_sim_time`/`start_type_description_service`). Confirmado
   ao vivo:
   ```
   $ ros2 param set /logger storage_failure_simulated true
   Setting parameter failed: Invalid access to undeclared parameter(s): []
   exit code: 0
   ```
   O `ros2 param set` falha de fato, mas o código de saída continua `0` —
   é por isso que o `Given` deste cenário confere a string `"Set parameter
   successful"` na saída, não só o `returncode`.
2. **Não existe o tipo de registro `"PersistenceFailure"`.** Nenhum dos
   cinco handlers `receive_*` de `logger.py` verifica uma flag de falha;
   cada um sempre escreve seu `type` fixo (`"Status"`, `"Event"`,
   `"EnergyStatus"`, `"AdaptationCommand"`, `"Uncertainty"`). A string
   `"PersistenceFailure"` não aparece em lugar nenhum do arquivo.

### Por que não é regressão

O `DataAccess.cpp` do ROS1 nunca teve um mecanismo de falha de
armazenamento real — ele recebe `persist` e despacha para
`persistStatus`/`persistEvent`/`persistEnergyStatus`/`persistUncertainty`/
`persistAdaptation`, sem nenhum branch de falha. O teste do ROS1 nem
verificava isso de verdade: seu `Then` conferia `'fail' in
parse_topic_data('/persist')['content']` — uma checagem de pertencimento
numa tupla de strings, não uma substring — e isso batia com o `Status`
de rotina do hub central (`/g4t1`, ver seção acima), não com um registro de
falha de persistência. Este cenário pede um mecanismo que **nunca existiu em
nenhuma das duas versões** — é escopo novo introduzido pela reescrita do
Gherkin, não algo que o porte quebrou.

## Correção pendente para o cenário 2 (não aplicada)

O teste em si já foi corrigido dos dois lados (2026-09-12): tanto
`bsn_ros2/features/steps/test_data_persistance.py` quanto o
`test_data_persistance.py` do ROS1 (`bsn2`) agora fazem a asserção
*correta* — um registro `type == "PersistenceFailure"` cujo `source`/
identidade case com a leitura reportada — em vez dos cheques frouxos que
passavam por acidente. Isso está marcado `xfail` (não `skip`) nos dois
lados, com o motivo documentado inline. Falta só o código-fonte de
verdade; sem tocar nele agora, por instrução explícita — só documentando o
que seria necessário caso decidam construir isso:

1. **ROS2** — declarar um parâmetro `storage_failure_simulated` (booleano,
   default `false`) em `logger.py`; nos cinco handlers `receive_*`, checar
   essa flag e, se ativa, publicar `Persist.type = "PersistenceFailure"` em
   vez do tipo normal, referenciando a leitura que falhou.
2. **ROS1** — o equivalente em `Logger.cpp`/`DataAccess.cpp`: um jeito de
   armar a falha (já que não há parâmetro dinâmico nesse código C++) e um
   `type = "PersistenceFailure"` nos handlers `receiveStatus`/`receiveEvent`/
   etc. quando armado.

Isso é uma mudança de comportamento nova nos dois lados, não uma correção de
regressão: não existe um "jeito certo" de referência em nenhuma das duas
versões para copiar, já que nenhuma implementou isso de verdade. Antes de
construir, vale decidir se esse cenário deveria mesmo existir como está, ou
ser reescrito para o que as duas versões realmente fazem hoje (persistir
tudo incondicionalmente, sem branch de falha) — o mesmo tipo de decisão que
já foi tomada para o cenário equivalente em `managing_system.feature`.

## O que nenhuma versão faz (contexto para o cenário 1)

Nem o `logger.py` do ROS2 nem o `DataAccess.cpp` do ROS1 jamais guardaram o
valor bruto de uma leitura de sensor — só bateria e registros de
status/evento/adaptação. No ROS1,
`DataAccess::processTargetSystemData` só copia níveis de bateria
(`components_batteries["g3t1_1"] = msg->trm_batt`); os campos com os
valores de leitura de fato (`trm_data`, `oxi_data`, …) nunca são lidos por
ninguém. No ROS2, `logger.py` não assina nenhum tópico `sensor_data/<key>`
— as únicas cinco assinaturas que tem (`log_status`, `log_event`,
`log_energy_status`, `log_adapt`, `log_uncertainty`) encaminham o que
recebem para `/persist` sem alteração, mas nada alimenta a leitura bruta
nelas. É por isso que a cláusula literal "with the value reported" do
cenário 1 não é satisfeita por nenhuma das duas versões — e por que isso não
conta como regressão.
