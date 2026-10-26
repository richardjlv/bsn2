# injector.feature — gap analysis (ROS1 → ROS2 migration)

`injector.feature` describes the uncertainty-injection subsystem (ROS1's
`/injector`, publishing `/uncertainty_/g3t1_N` to each sensor and
`/log_uncertainty` to the system log). As of this writing, **that node has
not been ported to this ROS2 workspace** - same class of gap as
[`MANAGING_SYSTEM_GAPS.md`](MANAGING_SYSTEM_GAPS.md) documents for
`/enactor`/`/reli_engine`/`/data_access`: it isn't a rename, the node simply
doesn't exist yet.

## Evidence it's missing, not renamed

- Repo-wide search (`grep -ri "injector\|uncertainty"`, `find`, every
  package's `setup.py` `entry_points`) across every package in the
  workspace (`sensor`, `system_monitor`, `shared_components`, `patient`,
  `central_hub`) turns up no `injector` executable and no source file that
  publishes an `/uncertainty_*` or `log_uncertainty` topic. The only
  matches are:
  - [`system_monitor/logger.py:29-30`](../system_monitor/system_monitor/logger.py#L29-L30) -
    `logger` **subscribes** to `log_uncertainty` (`Uncertainty` message) and
    forwards anything it receives to `/persist` as `type="Uncertainty"`
    ([`logger.py:82-91`](../system_monitor/system_monitor/logger.py#L82-L91)).
    The consumer side was ported; nothing publishes there today.
  - `system_monitor/test/test_logger.py` and `sensor/test/test_sensor_ros2.py`
    reference "uncertainty" only in unit-test scaffolding/comments (e.g.
    `test_sensor_ros2.py:66`: *"ROS1 depended on uncertainty injection and
    status-fail signaling specific to the legacy sensor..."*), not in any
    production code path.
  - `central_hub/test/test_central_hub_charac.py` has a
    `test_uncertainty_like_influence_on_collection` unit test, but it
    simulates the *effect* of uncertainty on a datapoint locally - it does
    not exercise a real `/uncertainty_*` topic or node.
- No sensor (`sensor/sensor/sensor.py` and its `components/*.py`) subscribes
  to anything uncertainty-related; their only publishers are
  `collect_event`, `status`/`sensor_data/<key>` (`components/publishers.py`)
  and `energy_status` (`components/battery_manager.py`).

## ROS1 comparison — `@regression`, confirmed

Both scenarios are tagged `@regression` in `injector.feature`: the legacy
ROS1 suite (`bsn2/src/sa-bsn/target_system/components/component/test/
test_bdd/`) has the **exact same Gherkin** (`features/injector.feature`,
byte-for-byte identical text) implemented **without** `@xfail` or any skip -
`test_injector.py` (ROS1) has a real, working step:

```python
@then(parsers.parse('{node} should publish "{topic}" to <sensor>'))
def then_inject_uncertainty_to_sensor(context, node, topic, sensor):
    topic_name = SYSTEM_MAP[topic] + SYSTEM_MAP[sensor]
    check_topic_inbound_from_node(context, topic_name, SYSTEM_MAP[node])
```

`check_topic_inbound_from_node` (ROS1's shared `conftest.py`) does a real
`is_node_publishing_to_topics` check - the same kind of check
`test_target_system.py`'s already-passing scenarios do in ROS2. This is not
a weak/generic assertion, and it was never marked `@xfail`: `/injector`
existed and was really wired to the six sensors and to `/logger` in ROS1.
Its total absence in this ROS2 port is a genuine regression, not a new or
previously-unverified requirement.

## Scenario-by-scenario status

| Scenario | Status |
|---|---|
| The uncertainty injector is wired to each body sensor (Outline, 6 examples) | 🔴 Blocked - no `/injector` node, no `/uncertainty_<node>` publisher on either end |
| The uncertainty injector is wired to the system log | 🔴 Blocked - no `/injector` node; `logger`'s `log_uncertainty` subscription exists but nothing publishes to it |

Both are implemented in
[`pytest_steps/test_injector.py`](pytest_steps/test_injector.py) as real
topology checks (`ros2 node info`, same `_assert_wired`-style pattern as
`test_target_system.py`/`test_managing_system.py`) marked
`pytest.mark.xfail` - not skipped or removed - so a future port that adds
`/injector` will surface a green run instead of a silently-passing gap, and
until then the failure output states exactly which topic/node is missing.

## To unblock

Unlike `/enactor`, `/reli_engine` and `/data_access` - which turn out to
already exist in ported form under `src/bsn2_test/adaptation` (see
`MANAGING_SYSTEM_GAPS.md`) - the uncertainty injector has **no ROS2 port
anywhere in this workspace**: `bsn2_test` has no injector either, and its
`features/` directory has no `injector.feature`. This one is a genuine
from-scratch port of `bsn2/.../simulation/injector/src/Injector.cpp`, which
advertises `log_uncertainty` and one `uncertainty_/<component>` topic per
sensor (`Injector.cpp:9,22,85-86`).

Port an `/injector` node that:

1. Publishes on a per-sensor uncertainty channel (ROS1: `/uncertainty_/g3t1_N`
   per sensor; a ROS2-idiomatic namespaced equivalent, e.g.
   `/uncertainty/<key>`, would need each sensor to also gain a subscription
   it doesn't have today - see `sensor/sensor/sensor.py`).
2. Publishes `Uncertainty` messages on `log_uncertainty`, which `logger`
   already subscribes to and forwards into `/persist` unchanged.

Once it exists, both scenarios in `injector.feature` can be un-xfailed as-is
(no step-definition changes needed) - `pytest_steps/test_injector.py`
already resolves sensor topic names through
[`utils/interface_map.py`](steps/utils/interface_map.py)'s `SENSORS` map.
