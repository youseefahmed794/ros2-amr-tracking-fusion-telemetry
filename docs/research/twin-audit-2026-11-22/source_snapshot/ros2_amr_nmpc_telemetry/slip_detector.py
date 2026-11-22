"""ROS-independent wheel-vs-gyro yaw-rate disagreement detector (detection only)."""
import math
from collections import deque


class YawRateSlipDetector:
    """Alarm when the windowed mean of (wheel yaw rate - gyro yaw rate) leaves its baseline.

    For a non-slipping, correctly scaled wheel the residual has mean -gyro_bias.
    The baseline ("center") is a running mean during warm-up, then a slow
    exponential average that is only updated while the windowed residual stays
    well inside the alarm threshold.

    Limits: only yaw-rate disagreement is observed. The detector is blind while
    the robot is not turning, cannot separate wheel slip from a wheel-scale error
    or a gyro-bias change, and does not modify any estimator or controller.
    """

    def __init__(self, window=25, threshold=0.0196, warmup_samples=100,
                 center_alpha=0.001, confirm_samples=1):
        if int(window) != window or window < 1:
            raise ValueError('window must be a positive integer')
        if int(warmup_samples) != warmup_samples or warmup_samples < window:
            raise ValueError('warmup_samples must be an integer >= window')
        if int(confirm_samples) != confirm_samples or confirm_samples < 1:
            raise ValueError('confirm_samples must be a positive integer')
        if not (math.isfinite(threshold) and threshold > 0):
            raise ValueError('threshold must be finite and positive')
        if not (math.isfinite(center_alpha) and 0 < center_alpha <= 1):
            raise ValueError('center_alpha must be in (0, 1]')
        self.window = int(window)
        self.threshold = float(threshold)
        self.warmup_samples = int(warmup_samples)
        self.center_alpha = float(center_alpha)
        self.confirm_samples = int(confirm_samples)
        self.reset()

    def reset(self):
        """Forget all history and the learned baseline."""
        self._buffer = deque(maxlen=self.window)
        self._sum = 0.0
        self._count = 0
        self._over = 0
        self.center = 0.0
        self.score = 0.0
        self.alarm = False

    @property
    def samples(self):
        """Number of finite samples accepted so far."""
        return self._count

    def update(self, wheel_yaw_rate, gyro_yaw_rate):
        """Feed one sample pair [rad/s]; return True while the alarm is active."""
        residual = wheel_yaw_rate - gyro_yaw_rate
        if not math.isfinite(residual):
            return self.alarm
        if len(self._buffer) == self.window:
            self._sum -= self._buffer[0]
        self._buffer.append(residual)
        self._sum += residual
        self._count += 1
        if self._count <= self.warmup_samples:
            self.center += (residual - self.center) / self._count
            self._over = 0
            self.score = 0.0
            self.alarm = False
            return False
        self.score = self._sum / len(self._buffer) - self.center
        over = abs(self.score) > self.threshold
        self._over = self._over + 1 if over else 0
        self.alarm = self._over >= self.confirm_samples
        if abs(self.score) <= 0.5 * self.threshold:
            self.center += self.center_alpha * (residual - self.center)
        return self.alarm
