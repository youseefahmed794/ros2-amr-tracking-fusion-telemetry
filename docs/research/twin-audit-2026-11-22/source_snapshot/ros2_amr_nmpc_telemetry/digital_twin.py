"""ROS-independent slip-aware digital twin of the differential-drive AMR (simulation-validated only).

The twin mirrors the robot from three measured signals (wheel linear rate, wheel yaw rate,
raw gyro yaw rate) and identifies two parameters online: the gyro bias and, while a slip
alarm is active, a traction factor. It keeps its own dead-reckoned pose built from the
bias-corrected gyro yaw rate and the wheel linear rate scaled by the traction factor.

Assumption, inherited from the compensator it is built on: traction loss scales the linear
and the yaw velocity by the same factor. That holds in the simulator; real slip need not
behave that way. There is no absolute position measurement, so the pose drifts.
"""
import math
from collections import deque

from .slip_detector import YawRateSlipDetector


class AMRDigitalTwin:
    def __init__(self, threshold=0.030, confirm_samples=1, window=50,
                 encoder_std=0.03, s_min=0.3):
        if int(window) != window or window < 1:
            raise ValueError('window must be a positive integer')
        if not (math.isfinite(encoder_std) and encoder_std > 0):
            raise ValueError('encoder_std must be finite and positive')
        if not (math.isfinite(s_min) and 0 < s_min <= 1):
            raise ValueError('s_min must be in (0, 1]')
        self.detector = YawRateSlipDetector(threshold=threshold, confirm_samples=confirm_samples)
        self.window = int(window)
        self.prior = self.window * encoder_std ** 2
        self.s_min = float(s_min)
        self._cross = deque(maxlen=self.window)
        self._power = deque(maxlen=self.window)
        self.reset()

    def reset(self):
        self.detector.reset()
        self._cross.clear()
        self._power.clear()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.traction = 1.0
        self.bias = 0.0
        self.slip = False
        self.slip_events = 0
        self.slip_time = 0.0
        self._t_prev = None

    def update(self, t, wheel_v, wheel_w, gyro_w):
        """Feed one sample (seconds, m/s, rad/s, rad/s); returns a telemetry dict."""
        if not all(math.isfinite(value) for value in (t, wheel_v, wheel_w, gyro_w)):
            return self.telemetry(t, valid=False)
        # Reject replayed timestamps before changing any internal state.
        if self._t_prev is not None and t <= self._t_prev:
            return self.telemetry(t, valid=False)
        was_slipping = self.slip
        self.slip = self.detector.update(wheel_w, gyro_w)
        if self.slip and not was_slipping:
            self.slip_events += 1
        self.bias = -self.detector.center
        self._cross.append(wheel_w * (gyro_w + self.detector.center))
        self._power.append(wheel_w * wheel_w)
        if self.slip:
            s = (sum(self._cross) + self.prior) / (sum(self._power) + self.prior)
            self.traction = float(min(1.0, max(self.s_min, s)))
        else:
            self.traction = 1.0
        if self._t_prev is not None and 0 < t - self._t_prev <= 0.5:
            dt = t - self._t_prev
            v = self.traction * wheel_v
            w = gyro_w - self.bias
            self.x += v * math.cos(self.yaw + w * dt / 2) * dt
            self.y += v * math.sin(self.yaw + w * dt / 2) * dt
            self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
            if self.slip:
                self.slip_time += dt
        self._t_prev = t
        return self.telemetry(t, valid=True)

    def telemetry(self, t, valid=True):
        return {'t': t, 'valid': valid, 'x': self.x, 'y': self.y, 'yaw': self.yaw,
                'traction': self.traction, 'gyro_bias': self.bias, 'slip': self.slip,
                'residual_score': self.detector.score, 'slip_events': self.slip_events,
                'slip_time': self.slip_time}
