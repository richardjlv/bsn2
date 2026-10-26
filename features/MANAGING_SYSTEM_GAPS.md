# managing_system.feature — análise de gaps (migração ROS1 → ROS2)

`managing_system.feature` cobre o subsistema de confiabilidade/adaptação do
BSN (`/enactor`, `/reli_engine`, `/data_access`, todos presentes e rodando
no pacote `adaptation` do `bsn_ros2`, subidos pelo
[`emergency_detection_launch.py`](../central_hub/launch/emergency_detection_launch.py)).

As mudanças de código-fonte do ROS2 que este subsistema exigiu durante a
validação (parâmetros do `enactor`/`reli_engine`, o `plan()` do
`reli_engine`, a tradução de nomes BSN↔nó, o `log_adapt`) estão
consolidadas em [`ROS2_SOURCE_CHANGES.md`](ROS2_SOURCE_CHANGES.md).

**Situação documentada em setembro: 9 dos 11 cenários coletados passam; 2 estão em `xfail`**,
cada um por um motivo específico e verificado (ver `XFAIL_REASON_BY_SCENARIO`
em [`steps/test_managing_system.py`](steps/test_managing_system.py)):

| Cenário | Tag | Status |
|---|---|---|
| Sampling rate decreases after sensor reliability recovers (G4/G5) | `@regression` | 🔴 xfail — regressão confirmada; a correção do formato da exceção foi aplicada e **não resolveu**, causa restante desconhecida (ver "Estado após a correção") |
| No adaptation is triggered while sensor reliability stays above the threshold (G4/G5) | só `@xfail` | 🔴 xfail — confirmado que reproduz o comportamento legado do ROS1, não é gap (ver nota própria em `test_managing_system.py`) |

Todo o resto — os checks de topologia enactor/engine/logger, o outline do
coletor de log, o check da rota de reconfiguração, "sampling rate
increases", o orçamento de 10 segundos e "a failed adaptation attempt is
recorded" — passa numa pilha recém-subida.

Na coleta por feature de 22/10/2026, o cenário de 10 segundos falhou quando
executado **depois** do cenário longo de recuperação na mesma pilha. O log
mostra o último comando ao oxímetro antes do teste em 39,77 Hz; o controlador
descarta um novo comando quando o próximo valor calculado excede o limite de
40 Hz. Com o cenário temporal movido para antes do ensaio de recuperação,
uma reexecução da feature completa em 23/10/2026 terminou aprovada pelo
executor externo (`1 passed`, 158,14 s). A nova ordem mitigou a interferência observada nessa execução; não demonstra
independência dos cenários. O resumo externo não preserva a distribuição
individual de PASS/XFAIL/XPASS, e não houve nova coleta de cobertura. A
asserção temporal atual ainda observa a chegada de
**algum** comando após iniciar os status `fail`: ela não comprova, por si,
o cruzamento do limiar de confiabilidade nem o nexo causal do comando com
esses status. Esse oráculo continua a exigir fortalecimento.

## Regressão em aberto: "Sampling rate decreases after sensor reliability recovers (G4/G5)"

### Sintoma

A frequência de amostragem comandada para o oxímetro sobe durante a injeção
sustentada de status `'fail'`, como esperado, mas nunca volta a cair depois
que a injeção de falhas para e a confiabilidade se recupera.

Medido em 2026-09-19, numa rodada isolada (só esta stack no grafo, com o
`_publish_exception` já corrigido): o oxímetro recebe 68 comandos de
frequência que sobem **de forma monotônica de 1,29 Hz a 23,81 Hz, sem nenhuma
queda**, e o teste falha com *"No AdaptationCommand with freq < 9.570Hz (the
measured peak) arrived … in 30.0s after the oximeter's reliability
recovered"*. Ou seja, não é um platô: a frequência continua **subindo** durante
a janela de recuperação (descrição anterior, "estacionada num platô", não
bate com a medição).

### Confirmado contra uma execução real no ROS1 — é regressão de verdade, não é comportamento legado compartilhado

Rodei o próprio `test_bdd/test_managing_system.py` do `bsn2` contra uma
pilha ROS1 real (`rostest --reuse-master component
test_bdd/test_managing_system.launch --text`), com uma captura em paralelo
de `rostopic echo /log_adapt`. Resultado: **o cenário passa no ROS1**, de
verdade:

```
Pico de frequencia atingido antes da recuperacao: 25.605Hz
Persist confirmado: type=AdaptationCommand source=/enactor content=freq=1.000000
.
============== 10 passed, 1 xfailed, 5 warnings in 34.14 seconds ===============
```

A captura bruta de `/log_adapt` confirma isso de forma independente: a
frequência comandada para `/g3t1_1` sobe até um pico de `25.605...`Hz e
depois cai numa tendência sustentada e monotônica, passando por
`25.47 → 25.33 → 25.20 → ... → 20.89`Hz ao longo dos ~60 segundos
seguintes — uma queda real de quase 19%, não ruído. Isso derruba a hipótese
anterior registrada neste arquivo (de que a ordem de busca por coordenadas
do planejador do ROS2 prende estruturalmente todo sensor no teto, e que essa
limitação poderia ser compartilhada com o ROS1, só que não verificada lá).
Não é compartilhada: o ROS1 comprovadamente escapa dela.

### Defeito real, mas não a causa completa: `Controller._publish_exception` não falava o formato que `Engine.receive_exception` espera

> **Atualização 2026-09-19.** O defeito descrito abaixo é real e **já foi
> corrigido**, mas corrigi-lo **não faz o cenário passar** (ver "Estado após a
> correção"). Esta seção descreve corretamente o defeito, mas o título
> anterior — "Causa raiz" — era uma conclusão que a medição não sustentou.

O `plan()` do `reli_engine` decide qual termo da fórmula (um sensor ou o hub
central) acaba com um valor genuinamente ajustável e fracionário, contra os
que são empurrados até o próprio teto, percorrendo `r_vec` — a lista de
termos `R_` ativos — em ordem de prioridade. Todo termo `R_` começa com a
mesma prioridade padrão (50), então, sem nada para desempatar, a ordenação é
estável e a ordem de `r_vec` nunca muda: ela fica travada na própria ordem
dos termos na fórmula (todo sensor antes do hub central,
[`resource/model/reliability.formula`](../adaptation/resource/model/reliability.formula)),
prendendo para sempre o hub como o único termo que sobra fracionário e todo
sensor como um dos que são empurrados até o teto.

No ROS1, esse empate é desfeito ao longo do tempo. O `Controller.cpp`
publica um `archlib::Exception` sempre que a correção de um componente ficou
consistentemente numa única direção por 5 ciclos seguidos
([`Controller.cpp:147,154,238,245`](../../bsn2/src/sa-bsn/system_manager/enactor/src/Controller.cpp#L147)),
com `content` no formato `component+"=1"` ou `component+"=-1"` (por exemplo,
`"/g3t1_1=1"`). O `Engine::receiveException`
([`Engine.cpp`](../../bsn2/src/sa-bsn/system_manager/adaptation_engine/src/Engine.cpp))
faz o parse disso por `=`, ajustando `priority[component]` para cima ou para
baixo e limitando entre `[0, 100]`. Como cada componente acumula esses
sinais no seu próprio ritmo, a `priority` diverge entre os componentes ao
longo de uma execução, a ordenação de `r_vec` muda entre um ciclo de
planejamento e outro, e eventualmente um *sensor* — não sempre o hub — acaba
sendo o último da fila e sobra com um valor genuinamente fracionário e
redutível. Foi exatamente isso que a captura ao vivo acima mostrou
acontecendo com o oxímetro.

O porte para ROS2 tem a metade receptora disso corretamente ligada — o
[`receive_exception` de `base_engine.py`](../adaptation/adaptation/engines/base_engine.py#L165-L191)
é um porte fiel e funcional de `Engine::receiveException`, com o mesmo split
por `=` e o mesmo clamp — mas a metade emissora nunca produz uma string que
esse parser consiga entender:

```python
# adaptation/adaptation/enactor/controller.py:219
exception_msg.content = f"Component {component} has {value} exceptions"
```

A atualização em `receive_exception` só roda dentro de `if "=" in content:`
([`base_engine.py:169`](../adaptation/adaptation/engines/base_engine.py#L169)),
e essa frase nunca contém um `=`. O `self.priority` está declarado
corretamente, assinado corretamente e parseado corretamente — ele
simplesmente nunca é *escrito*, porque nada que chega em `/exception` bate
com o formato que o próprio parser espera. A ordem de `r_vec` fica congelada
no valor padrão de `priority` de cada processo pela vida inteira da
execução, e é por isso que o hub é *sempre* o termo que sobra redutível e
nenhum sensor nunca tem a sua vez.

### Correção proposta (aplicada em 2026-09-19; ver o resultado abaixo)

Trocar `Controller._publish_exception`
([`controller.py:215-221`](../adaptation/adaptation/enactor/controller.py#L215-L221))
para publicar no mesmo formato `<componente>=<valor>` que o ROS1 usa e que o
próprio `receive_exception` deste porte já espera, por exemplo:

```python
exception_msg.content = f"{component}={value}"
```

Essa é uma mudança de comportamento ao vivo no laço de adaptação
compartilhado, não só deste cenário — todo cenário G4/G5 passa pelo mesmo
`plan()` — então a suíte inteira deve ser rodada de novo depois de aplicar,
não só este cenário.

### Estado após a correção (2026-09-19): necessária, mas não suficiente

A correção está aplicada (não commitada) e foi validada ao vivo. Resultado:

- **O mecanismo passou a funcionar.** Na suíte `managing_system` inteira, o
  `enactor` publicou 119 exceções e o `reli_engine` aplicou 119 atualizações
  de prioridade (`Updated priority: R_… = 51/52`); antes da correção esse
  número era zero, porque nada que chegava em `/exception` continha `=`.
- **O cenário continua `xfail`** (não `XPASS`), tanto na suíte inteira
  (`9 passed, 2 xfailed`) quanto isolado com `--runxfail` (sintoma na seção
  "Sintoma"). A hipótese de que o formato da exceção era a causa raiz está,
  portanto, **refutada como explicação completa**.
- **Padrão observado (reproduzido em duas rodadas independentes):** todos os
  7 componentes (6 sensores e o hub) emitem `=1` **no mesmo instante, a cada
  5 s**, sempre com valor `+1` e nenhuma vez `-1` (rodada isolada: 13 exceções
  por componente, 91 no total; suíte inteira: 17 por componente, 119). Como
  todos sobem juntos, as prioridades nunca divergem entre componentes — o que
  mantém a ordem de `r_vec` empatada, que era exatamente o mecanismo que a
  hipótese anterior queria destravar.
- **Não é o reset de 10 s.** `Engine.body()` recarrega a fórmula e
  reinicializa `priority` a cada 10 s no ROS2 (por isso só se veem valores 51 e
  52), mas o ROS1 faz o mesmo (`Engine.cpp:170-176`): é porte fiel, não a
  diferença.
- **Divergência de código observada, sem prova de causalidade.**
  `Controller.cpp` (ROS1) decide adaptar com
  `error > margin*r_ref || error < margin*r_ref`, que é verdadeira para quase
  todo `error` (só falha se for exatamente igual a `margin*r_ref`); o porte
  usa `abs(error) > margin*r_ref` (`controller.py:99`), com faixa morta de
  verdade e um ramo de decremento alcançável (que pode gerar `-1`). Nesta
  medição os passos do oxímetro são de ~0,3–0,4 Hz; com `kp=150`
  (`new_freq = freq + kp/100 * error`) isso implica `error` em torno de 0,2 a
  0,27, muito acima da margem de 2% — inferência, pois não registrei
  `r_curr`/`r_ref` diretamente. Nessa hipótese a faixa morta **não** explica
  este cenário; fica registrada apenas como divergência a ter em mente.

**Causa restante: desconhecida.** O que falta explicar é por que, no ROS1, a
confiabilidade-alvo do oxímetro fica abaixo da medida (erro negativo, e a
frequência cai), enquanto no ROS2 o erro permanece positivo mesmo depois da
recuperação. Próximo passo sugerido: comparar, ciclo a ciclo, `r_curr`/`r_ref`
do oxímetro nos dois sistemas.

## O outro `xfail`: "No adaptation is triggered while sensor reliability stays above the threshold (G4/G5)"

Não é regressão. O controlador do SA-BSN é um otimizador contínuo
(`kp=150`, limites `[0.1, 40]`Hz, `stability_margin=0.02`): com `r_curr` já
em ou perto de 1.0, ele continua emitindo pequenos comandos corretivos em
vez de ficar em silêncio. Esse é o comportamento legado do ROS1, reproduzido
fielmente pela migração — ver `CONTROLLER_NEVER_SILENT_REASON` em
`steps/test_managing_system.py`. Mantido `xfail` de propósito: um pass aqui
significaria que o comportamento do controlador mudou silenciosamente.

## Isolamento e reexecuções complementares de 23/10/2026

O executor passou a usar uma montagem independente por cenário de adaptação. Na campanha `ros2_20261023T122757Z_s9czcx_k`, a ordem dos grupos inclui recuperação antes do temporal, com processos novos: o temporal aprovou. A execução completa teve 8 PASS, 2 XFAIL e 1 FAIL. As falhas esperadas ocorreram nas asserções de recuperação e silêncio; a falha inesperada foi um timeout de 5 s ao publicar o estímulo do cenário de registro via CLI. O publicador foi substituído por um participante rclpy persistente, preservando conteúdo, cinco publicações e asserção sobre `/persist`. A reexecução individual `ros2_20261023T123635Z_cmd7psup` aprovou; as reexecuções posteriores estão registradas abaixo.

O temporal também aprovou sozinho em `ros2_20261023T122618Z_mqbypydy`. A primeira tentativa `ros2_20261023T122341Z_7sj4kj7u` não executou cenários devido ao timeout da preparação interna por CLI. Com `--direct-readiness`, a fixture interna agora consulta serviços de ciclo de vida, como o executor externo. O cenário só começa após evidência de atividade do controlador; isso ainda não demonstra confiabilidade estável ou nexo causal com o estímulo.

XML individuais, traces, cobertura e limitações estão no [relatório de isolamento](https://github.com/richardjlv/bsn_traces/tree/main/diagnosticos/managing_system_isolado_2026-10-23).

## Situação conferida em 24/10/2026

As reexecuções posteriores ainda falharam na preparação e na descoberta de interfaces. A última campanha (`ros2_20261023T185710Z_oitpv2vt`) terminou por timeout global: 2 PASS, 2 XFAIL e 6 FAIL internos, sem resultado do registro. Foi preparada uma espera de cinco segundos pelos endpoints exigidos nas asserções de topologia, ainda sem validação ROS por falta de espaço. As duas marcações XFAIL agora aceitam apenas a exceção das asserções funcionais, sem ocultar erros de preparação. Os detalhes e todas as tentativas estão no relatório de isolamento vinculado acima.

## Revalidação de 25/10/2026

A campanha `ros2_20261025T054442Z_l7kfhjkk` concluiu os 11 casos com 9 PASS e 2 XFAIL, sem falhas inesperadas. As falhas esperadas ocorreram nos Then de recuperação e silêncio. A espera por endpoints foi validada em topologia isolada e na rodada completa. O temporal passou após recuperação em nova pilha, e o registro passou. A tentativa anterior `051720Z` ficou incompleta e não foi somada. Permanecem limitações de causalidade, observação de confiabilidade e exceções de encerramento. [Relatório e evidências](https://github.com/richardjlv/bsn_traces/tree/main/diagnosticos/managing_system_2026-10-25).
