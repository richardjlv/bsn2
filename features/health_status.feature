# Propriedades e resultados: src/tg/apendices/apendice_b_propriedades_bdd.md
Feature: Patient Health Status (BSN-P10)

	A patient health status is available whenever a sensor reports a reading,
	and stops being produced while the central hub is unable to process
	incoming readings.

	@behavior @bsn-p10
	# Rastreabilidade BDD08: BSN-P10. Retomada de P10.
	Scenario: A patient health status is available after a sensor reports a reading (BSN-P10)
		Given the patient is being monitored by the thermometer
		When the thermometer reports a new vital sign reading
		Then a patient health status derived from that reading should be available

	# D-healthstatus-01: worded differently from the happy-path When on purpose.
	# With the hub unable to process readings, /target_system_data never
	# publishes, so this step needs a bounded wait instead of the shared
	# "{sensor} reports a new vital sign reading" step (which blocks until a
	# fixed line count arrives - see pytest_steps/test_check_bsn.py's "the ECG
	# sensor and the thermometer report new vital sign readings" for the same
	# reasoning).
	@behavior @sad-path @bsn-p10
	# Rastreabilidade BDD09: BSN-C04. Variante local de indisponibilidade, relacionada a P10.
	Scenario: No patient health status is produced when the central hub cannot process a reading (BSN-P10)
		Given the central hub is unable to process incoming readings
		When the thermometer reports a reading
		Then no new patient health status should be produced for that reading
