# Propriedades e resultados: src/tg/apendices/apendice_b_propriedades_bdd.md
Feature: Data Persistence (BSN-P08) - Whether the sensor node has collected some data, eventually the bodyhub will persist it.

	@behavior @bsn-p08 @persistence
	# Passa: um registro do sensor chegando ao /persist (o que o teste ROS1 verificava) funciona aqui.
	# A clausula "with the value reported" nenhuma versao satisfaz (o ROS1 nunca guardou o valor);
	# nao e regressao, e escopo novo. Ver DATA_PERSISTANCE_GAPS.md.
	# Rastreabilidade BDD06: BSN-P08. Retomada de P8; enunciado atual exige recuperação do valor.
	Scenario: A vital sign reading collected by a sensor is persisted in the knowledge repository (BSN-P08)
		Given the patient is being monitored by the thermometer
		When the thermometer reports a new vital sign reading
		Then that reading should be retrievable from the knowledge repository with the value reported

	@behavior @sad-path @bsn-p08 @persistence @gap
	# xfail: gap nas DUAS versoes. Replay do ROS1 (2026-09-12): o kill do /logger nunca funciona e o
	# "fail" vinha do /g4t1 (hub central), sem relacao com persistencia. Nenhuma versao tem
	# branch de falha de armazenamento
	# Rastreabilidade BDD07: BSN-C03. Complementar de falha de armazenamento, relacionada a P8.
	Scenario: A persistence failure is recorded when the knowledge repository cannot store a reading (BSN-P08)
		Given the knowledge repository is experiencing storage failures
		When the thermometer reports a new vital sign reading
		Then a persistence failure record identifying that reading should be available in the system log
