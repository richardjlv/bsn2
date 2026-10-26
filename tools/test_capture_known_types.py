"""Check typed capture without requiring a live ROS graph."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import pytest

steps = Path(__file__).resolve().parents[1] / 'features/steps'
sys.path.insert(0, str(steps))
spec = importlib.util.spec_from_file_location('capture_parsers', steps / 'utils/parsers.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

@pytest.mark.parametrize('topic,typename', [
    ('/target_system_data', 'TargetSystemData'),
    ('/sensor_data/thermometer', 'SensorData'),
    ('/persist', 'Persist'),
])
def test_known_type_without_graph(topic, typename):
    fake = type(typename, (), {'__slots__': ('_header', '_value')})
    with patch.object(module.subprocess, 'run', side_effect=AssertionError('Must not query graph')), patch.object(module.importlib, 'import_module', return_value=SimpleNamespace(**{typename: fake})):
        assert '_value' in module.get_message_attributes(topic)
    cmd = module._topic_echo_command(topic, "m.source == 'thermometer_node'")
    assert cmd[-2:] == [topic, 'bsn_interfaces/msg/' + typename]
    assert cmd[cmd.index('--filter') + 1] == "m.source == 'thermometer_node'"

def test_unknown_type_failure_is_not_silence():
    with patch.object(module.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stderr=b'unknown topic')):
        with pytest.raises(RuntimeError, match='Cannot discover'):
            module.get_message_attributes('/unknown')
