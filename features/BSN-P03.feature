# Propriedades e resultados: src/tg/apendices/apendice_b_propriedades_bdd.md
Feature: Emergency detection (BSN-P03) A high-risk vital sign reading is escalated to an emergency within 250 ms.
	
	@timing @bsn-p03
	# Regressao encontrada e corrigida por configuracao (por isso ja nao e
	# @xfail nem @regression - historico mantido de proposito): o hub so
	# publicava /target_system_data a 2Hz, entao a latencia de uma leitura
	# ficava uniforme entre 0 e ~500ms, e um orcamento de 250ms no meio dessa
	# faixa passava ou falhava por sorte a cada execucao.
	#
	# Causa raiz: nao era arquitetural (o hub ROS1 faz o mesmo poll), so a
	# cadencia. O legado rodava a 6Hz (g4t1.launch), ciclo de ~167ms, dentro
	# do orcamento; a porta ROS2 ficou em 2Hz (~500ms), fora dele. Correcao:
	# `parameters=[{"frequency": 6.0}]` no central_hub_node dos dois launches
	# (o default do proprio no continua 2.0 -
	# central_hub/components/config_manager.py:9 - a mudanca e so de launch).
	# Rastreabilidade BDD01: BSN-P03. Retomada de P3, com fronteira de observação adaptada.
	Scenario: An emergency is reported within 250 ms of a high-risk reading (BSN-P03)
		Given the patient is being monitored by the thermometer
		When the thermometer reports a vital sign reading in the high-risk range
		Then the central hub should report an emergency within 250 ms of that reading

	@timing @sad-path @bsn-p03
	# Ja foi @xfail. Deixou de ser porque a propriedade que ele afirma - o hub
	# cumprir os 250ms com o fluxo do sensor saturado - passa hoje; o que nao
	# funcionava era o MECANISMO que o Given usava para criar a carga, nao a
	# propriedade sob teste.
	#
	# Carga atual: o proprio teste publica 20Hz em sensor_data/thermometer e
	# depois troca o fluxo para alto risco (ver _SensorLoad em
	# pytest_steps/test_bsn_p03.py) - o hub nao distingue quem publica no
	# topico, mesma premissa do cenario acima. Limite honesto: carrega o HUB,
	# nao o processo do sensor.
	#
	# O que continua quebrado (regressao real, coberta em outro lugar):
	# dirigir o sensor por AdaptationCommand em reconfigure_<node> - o que o
	# Given fazia antes - nao muda a taxa nesta porta.
	# AdaptationHandler.process_adaptation_command
	# (shared_components/adaptation_handler.py) grava a nova frequencia em
	# `self.frequency` em vez de `self.node.config.frequency`, e
	# Sensor.spin_sensor (sensor/sensor.py) cria o `rate` uma unica vez no
	# inicio do loop - o rclpy.Rate nao muda de periodo depois. Confirmado
	# como regressao: o Sensor.cpp do ROS1 reconstruia o `ros::Rate` a cada
	# iteracao. Coberta - @xfail e @regression - por managing_system.feature,
	# "Sampling rate decreases after sensor reliability recovers (G4/G5)" (ver
	# MANAGING_SYSTEM_GAPS.md #6); nenhuma cobertura foi perdida tirando o
	# mecanismo daqui.
	#
	# Alternativa mais fiel, mexendo em launch: dar
	# `parameters=[{"frequency": 20.0}]` ao thermometer_node faz o sensor real
	# gerar a carga desde o startup, sem depender da adaptacao em runtime.
	# Rastreabilidade BDD02: BSN-P03. Variante local sob carga de P3.
	Scenario: An emergency is reported within 250 ms even while the sensor is under sustained load (BSN-P03)
		Given the thermometer is reporting low-risk readings at its maximum sampling rate
		When the thermometer reports a vital sign reading in the high-risk range
		Then the central hub should report an emergency within 250 ms of that reading
