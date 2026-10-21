"""Reject delayed, replayed, out-of-order and future sensor timestamps."""
import math


class TimestampGate:
    def __init__(self, max_age=0.25, future_tolerance=0.02):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_age must be positive')
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.last = None

    def accept(self, stamp_ns, now_ns):
        age = (now_ns-stamp_ns)*1e-9
        if stamp_ns <= 0 or age > self.max_age or age < -self.future_tolerance:
            return False
        if self.last is not None and stamp_ns <= self.last:
            return False
        self.last = stamp_ns
        return True
