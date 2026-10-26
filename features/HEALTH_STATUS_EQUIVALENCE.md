# Revisão local do oráculo de health_status — 26/10/2026

Estado: alinhamento das asserções, ainda sem validação integrada pareada.
Nada desta revisão foi enviado ao remoto. Os resultados históricos permanecem
associados aos testes anteriores, sem alteração retroativa.

## Preservação

Cenários anteriores salvos nos commits ROS 1 `5b2406f8` e ROS 2 `0f8813e`.
Demais alterações locais de fontes e relatórios versionados preservadas na
branch `backup/antes-equivalencia-2026-10-26`: ROS 1 `5e660d54`, ROS 2 `fbef219`.
O executor aninhado ros_pytest tem backup local `201c4ad` na mesma branch.
Campanhas e diretórios de preparação não versionados permanecem no disco;
não foram apagados, movidos nem incluídos nesses commits de código.
A branch de trabalho é `testes/equivalencia-health-status-2026-10-26`.

## Mudança

Antes, ROS 1 exigia mais de um valor textual de patient_status, sem validar
sua faixa; ROS 2 exigia percentuais válidos, sem variação. Agora ambos usam
cópias idênticas de health_status_oracle.py. O positivo exige uma leitura
numérica finita do termômetro e estados numéricos entre 0 e 100, com pelo
menos dois valores numericamente distintos. O negativo exige leituras
válidas do termômetro e ausência de estado de saúde.

O mapeamento da leitura é `data` no ROS 1 e `sensor_datapoint` no ROS 2.
Não houve mudança de produção nesta revisão. A condição de variação foi
preservada da caracterização legada; não constitui requisito universal para
entradas constantes. A redação do Then explicita a observação, sem afirmar
que já se identificou a resposta causada por uma leitura específica.

## Validação e pendências

O teste local `src/ferramentas_traces/test_health_status_equivalence.py`
verificou 26 combinações (13 em cada versão) e identidade dos dois arquivos:
27 aprovações. Entre os contraexemplos estão ausência de estímulo, ausência
de resposta, saída indevida no caso negativo, NaN, valores fora da faixa e
variação apenas de formatação textual. Isso testa o oráculo, não o sistema ROS.

A tentativa integrada ROS 2 `ros2_20261026T132146Z_bhlt9626` terminou pelo
limite externo de 300 segundos (`timed_out=true`, exit_code=-9), antes de
qualquer início de cenário ou passo. Não gerou JUnit interno nem coverage
combinável. O log e o manifesto foram preservados em `src/campanhas_traces`.
Não contabilizar como aprovação nem como reprovação de comportamento.
Este contêiner só tem Jazzy; ROS 1 requer execução no ambiente Melodic.

Ainda não há equivalência experimental demonstrada: as preparações, janelas
de captura e trajetórias do simulador continuam diferentes. O ROS 1 usa
captura compartilhada com limite de linhas e espera de segurança; o ROS 2
usa janelas de tempo. O cenário negativo também utiliza passos de captura
diferentes. A próxima etapa é estabelecer estímulos controlados comuns,
observar prontidão antes de interromper o hub e relacionar as leituras à
saída, além de uniformizar a janela. Só então novas campanhas poderão
sustentar uma afirmação de equivalência para esse recorte de P10.

## Diagnóstico posterior e ajuste dos limites internos

A campanha `ros2_20261026T210057Z_e35a75vc`, sem coverage e com limite
externo de 1200 s, terminou com rc=1, timed_out=false e dois casos internos
reprovados. O positivo falhou no When: consulta do tipo de
`/target_system_data` excedeu 5 s. O negativo falhou no Given: pkill excedeu
5 s. Nenhum chegou ao Then. O executor concluiu em aproximadamente 467 s.

Depois dessa campanha, os limites de subprocess.run para a consulta de tipo
e para pkill foram elevados de 5 para 30 s. As janelas de captura e os
critérios do oráculo não foram alterados. O ajuste é compartilhado com
outras features que usam esses auxiliares. Sua validação integrada ainda
depende de nova execução. A retirada de coverage não eliminou as falhas
nessa tentativa; isso não determina a causa raiz da lentidão.

Na campanha `ros2_20261026T213006Z_kgkmhi7a`, os dois cenários falharam
no When, por timeout de 30 s de `ros2 topic type /target_system_data`.
O Given de indisponibilidade do hub terminou desta vez. Nenhum Then foi
executado e o limite externo não foi atingido.

A captura agora usa tipos explícitos para /target_system_data (TargetSystemData),
/sensor_data/thermometer (SensorData) e /persist (Persist), todos de
bsn_interfaces. O mesmo tipo é passado ao comando echo, permitindo observar
inclusive a ausência de publicador sem depender da descoberta do tipo.
Os demais tópicos mantêm descoberta dinâmica, com erro explícito se falhar.
Quatro verificações isoladas aprovaram: três tipos conhecidos sem consulta
ao grafo e propagação de erro de descoberta para tópico desconhecido.
As janelas e asserções permanecem iguais. Essa alteração de instrumentação
ainda exige nova validação integrada; não aprova os cenários retroativamente.

## Validação integrada ROS 2 após o ajuste de captura

Campanha `ros2_20261026T215554Z__keo0cuh`, sem coverage: rc=0,
timed_out=false, complete=true; JUnit interno com 2 aprovações, nenhuma
falha, erro ou caso ignorado. Os traces registram os seis passos e término
dos dois Then revisados (validade/variação e ausência com leituras presentes).
Tempos individuais no JUnit: 16,242 s e 5,259 s. A rodada demonstra aprovação
dos critérios atuais no ROS 2 sob essas condições, sem constituir coleta
de cobertura ou comparação integrada pareada com ROS 1. O uso de tipos
explícitos eliminou nesta execução o ponto de falha de descoberta visto
nas tentativas anteriores; a causa raiz da lentidão da consulta permanece
não determinada. As demais limitações de estímulo e janela continuam válidas.
