# Coleta ROS 2: pacotes, BDD e cobertura conjunta

Com o ambiente ROS 2 e o workspace compilado/carregado, execute:

```bash
bash src/bsn_ros2/coverage.sh
```

O caminho acima pressupõe a raiz `/ros_ws`. O script também pode ser chamado de
outro diretório pelo caminho absoluto. Use `--output-root /caminho` para mudar o
diretório das coletas. Não é necessário apagar relatórios anteriores.

Antes da campanha, compile a versão atual dos pacotes e carregue o seu
`install/setup.bash`. Os executáveis, interfaces e arquivos de lançamento
instalados precisam corresponder ao checkout. O coletor prioriza os módulos
Python deste checkout por `PYTHONPATH`; isso não atualiza arquivos de lançamento
ou interfaces geradas. Não misture o workspace com a cópia `bsn2_test`.

## Artefatos por campanha

Uma pasta nova `coverage_runs/run_<UTC>_<identificador>/` contém:

- `packages/`: testes das pastas `test` dos seis pacotes Python, executados
  sequencialmente, com XML individual por pacote e logs completos.
- `bdd/`: execução dos oito lançamentos de teste de `features/test` e cobertura
  da fase integrada. `internal_junit/` conserva um XML por arquivo de passos,
  incluindo nomes parametrizados e marcações de falha esperada. `features.xml`
  é o resultado dos executores externos, contado separadamente.
- `combined/`: união dos perfis das duas fases, sem somar percentuais ou contar
  uma linha compartilhada duas vezes.
- Em cada uma dessas três pastas: `html_raw/index.html`,
  `html_production/index.html`, `coverage_raw.xml/json` e
  `coverage_production.xml/json`, além do banco `.coverage` combinado.
- `manifest.json`: comandos, códigos de retorno, resultados por caso,
  relatórios internos BDD ausentes, arquivos com linhas executadas e indicação
  de coleta de ramos. O manifesto começa como incompleto e é atualizado durante
  a execução; falhas de teste não são convertidas em aprovação pela geração do HTML.
- `sources.tar.gz`, hashes das fontes e `dependencies.log`: fontes, configurações
  e dependências para identificar a campanha. Não substituem uma imagem do ambiente
  nem um arquivo completo da instalação ROS.

## Escopo e instrumentação

As fontes medidas são os módulos próprios de `adaptation`, `central_hub`,
`patient`, `sensor`, `shared_components` e `system_monitor`. O bruto inclui
auxiliares dentro desses módulos; o recorte de produção exclui `test/`, `tests/`,
`test_components/`, `*_tests.py` e `setup.py`. Dependências ROS, interfaces geradas
e o próprio executor BDD não pertencem a esse denominador de código de produção.
O relatório cobre as fontes instrumentáveis desse escopo, não todo comportamento
possível do sistema nem código C++ externo.

A configuração registra linhas e ramos. O percentual global mostrado pelo
coverage.py com ramos ativos combina oportunidades de linhas e ramos; para
comparar **cobertura de linhas** com o ROS 1, use separadamente `lines-covered`
e `lines-valid` do XML. Ramos Python não equivalem a regiões LLVM.

As opções locais `--cov` dos `pytest.ini` são substituídas nesta campanha, para
evitar que diferentes configurações sobrescrevam os bancos ou interrompam a
instrumentação. O processo Python e seus descendentes recebem um hook temporário
de `coverage.process_startup()`, com arquivos paralelos, conforme a documentação
de [subprocessos do coverage.py 7.4.4](https://coverage.readthedocs.io/en/7.4.4/subprocess.html).
Não há edição de `site-packages`. Um `sitecustomize` preexistente é executado antes
do hook da campanha. Os dados paralelos são preservados após a combinação.

É necessário conferir `executed_files` e os avisos nos logs. Um processo morto
com SIGKILL ou encerramento abrupto pode não gravar seu perfil; iniciar o hook
não garante a captura final de todos os nós. Encerramento normal e tratamento
de SIGTERM são contemplados pela configuração. Cobertura ausente não deve ser
interpretada automaticamente como ausência de exercício do código.

## Interpretação e comparação

A fase `packages` não significa automaticamente “testes apenas para cobertura”:
ela também inclui testes de comportamento, contratos, classes e infraestrutura.
A fase BDD tampouco é apenas textual: seus nós contribuem para a cobertura do
código de produção. Para comparar por finalidade, classifique os casos na matriz,
além de usar a separação por montagem oferecida pelos relatórios.

O total de casos internos reúne os casos dos pacotes e os cenários BDD,
**excluindo os executores externos BDD**. Os `xfail` não são aprovações.
JUnit pode representar XPASS não estrito como aprovação; os logs `-rA` devem ser
consultados antes de concluir que uma expectativa previamente falha foi satisfeita.

Os resultados internos podem também ser coletados fora desta campanha definindo
`BSN_BDD_RESULTS_DIR` para um diretório novo antes de executar os testes BDD.
Cada relatório usa o nome do arquivo de passos; não execute duas cópias do mesmo
arquivo simultaneamente nesse diretório.

Esta coleta não corrige o aviso ROS 1 `functions have mismatched data`:
esse aviso é de compatibilidade entre perfis e mapeamentos de objetos LLVM,
não de incompatibilidade entre testes de caracterização e testes estruturais.
O diagnóstico da causa precisa ocorrer no ambiente e nos artefatos ROS 1.
