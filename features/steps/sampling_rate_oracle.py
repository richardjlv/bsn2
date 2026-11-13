"""Observation criterion shared by ROS 1 and ROS 2 adaptation tests."""
import math
import threading

WINDOW_S = 6.0
MIN_MESSAGES = 4
MIN_RELATIVE_CHANGE = 0.10


def summarize_rate(stamps):
    assert len(stamps) >= MIN_MESSAGES, 'Insufficient sensor messages: {}'.format(len(stamps))
    intervals = [b - a for a, b in zip(stamps, stamps[1:])]
    assert all(x > 0 and not math.isinf(x) and not math.isnan(x) for x in intervals), 'Invalid observation intervals'
    ordered = sorted(intervals)
    middle = len(ordered) // 2
    median = ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) / 2.0
    return {'hz': 1.0 / median, 'messages': len(stamps),
            'span_s': stamps[-1] - stamps[0], 'timestamps': stamps}


def assert_rate_changed(before, after, direction):
    assert direction in ('increase', 'decrease')
    ratio = after['hz'] / before['hz']
    passed = ratio > 1.0 + MIN_RELATIVE_CHANGE if direction == 'increase' else ratio < 1.0 - MIN_RELATIVE_CHANGE
    assert passed, 'Sensor rate did not {} by more than 10%: before={:.3f} Hz, after={:.3f} Hz'.format(direction, before['hz'], after['hz'])


def observe_rate(subscribe, poll, clock):
    stamps = []
    lock = threading.Lock()

    def receive(_message):
        with lock:
            stamps.append(clock())

    unsubscribe = subscribe(receive)
    try:
        deadline = clock() + 10.0
        while clock() < deadline:
            poll()
            with lock:
                if stamps:
                    break
        with lock:
            assert stamps, 'Sensor did not publish during discovery'
            stamps[:] = []
        deadline = clock() + WINDOW_S
        while clock() < deadline:
            poll()
        with lock:
            result = summarize_rate(list(stamps))
        print('sensor_rate_observation: {}'.format(result))
        return result
    finally:
        unsubscribe()
