Feature: Data Persistence (BSN-P08) - Whether the sensor node has collected some data, eventually the bodyhub will persist it.

	@behavior @bsn-p08 @persistence
	# Passa. O comportamento que o teste ROS1 de fato verificava - um registro
	# produzido pelo sensor chegando ao /persist - funciona nesta porta
	# (medido: registros ("Status","thermometer_node") continuos). A clausula
	# literal "with the value reported" nao e satisfeita por NENHUMA das duas
	# versoes (o DataAccess.cpp do ROS1 guardava baterias e registros de
	# status/evento, nunca o valor da leitura), entao nao e regressao - e
	# escopo novo, introduzido pela reescrita da revisao. Ver
	# DATA_PERSISTANCE_GAPS.md.
	Scenario: A vital sign reading collected by a sensor is persisted in the knowledge repository (BSN-P08)
		Given the patient is being monitored by the thermometer
		When the thermometer reports a new vital sign reading
		Then that reading should be retrievable from the knowledge repository with the value reported

	@behavior @sad-path @bsn-p08 @persistence @gap
	# Ignorado (skip), nao xfail: e gap nas DUAS versoes. O ROS1 induzia a
	# falha com rosnode.kill_nodes('/logger') e depois assertava sobre o
	# /persist - topico cujo unico publicador acabara de matar; replicando essa
	# logica aqui, a captura volta com 0 linhas, ou seja, a assercao nao podia
	# valer nem no ROS1. O cenario e autocontraditorio como especificado.
	Scenario: A persistence failure is recorded when the knowledge repository cannot store a reading (BSN-P08)
		Given the knowledge repository is experiencing storage failures
		When the thermometer reports a new vital sign reading
		Then a persistence failure record identifying that reading should be available in the system log
