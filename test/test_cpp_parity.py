"""Compare stateful C++ control results to the Python benchmark core."""
import os
import subprocess
import numpy as np
import pytest
from ros2_amr_nmpc_telemetry.core import Tracker


def test_cpp_matches_python_with_saturation_and_reset():
    executable = os.environ.get('TRACKER_PROBE')
    if not executable:
        pytest.skip('Built tracker_probe required; run colcon test')
    rng = np.random.default_rng(23)
    inputs, expected = [], []
    tracker = Tracker()
    for k in range(3000):
        pose = rng.normal(0, 10, 3)
        dt = -0.1 if k % 101 == 0 else 0.01
        t = k*0.01
        inputs.append(' '.join(map(str, [*pose, t, dt])))
        expected.append(tracker.step(pose, t, dt))
    result = subprocess.run([executable], input='\n'.join(inputs)+'\n',
                            text=True, capture_output=True, check=True, timeout=10)
    actual = np.array([[float(v) for v in row.split()]
                       for row in result.stdout.splitlines()])
    np.testing.assert_allclose(actual, expected, rtol=1e-11, atol=1e-11)
