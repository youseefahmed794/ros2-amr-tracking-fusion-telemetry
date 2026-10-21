"""ROS-independent planar velocity fusion and bounded trajectory tracking."""
import math
import numpy as np


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def reference(t):
    scale, w = 3.0, 0.25
    x, y = scale * math.sin(w*t), scale * math.sin(w*t) * math.cos(w*t)
    dx, dy = scale*w*math.cos(w*t), scale*w*math.cos(2*w*t)
    ddx, ddy = -scale*w*w*math.sin(w*t), -2*scale*w*w*math.sin(2*w*t)
    speed = math.hypot(dx, dy)
    return x, y, math.atan2(dy, dx), speed, (dx*ddy-dy*ddx)/max(speed**2, 1e-6)


class VelocityEKF:
    """State [x,y,yaw,v,w]; velocity-only fusion cannot observe absolute pose."""
    def __init__(self, q_pos=0.01, q_yaw=0.005, r_odom=0.05, r_imu=0.002):
        if not all(math.isfinite(v) and v > 0 for v in (q_pos, q_yaw, r_odom, r_imu)):
            raise ValueError('Noise variances must be finite and positive')
        self.x = np.zeros((5, 1))
        self.P = np.eye(5)*0.1
        self.Q = np.diag([q_pos, q_pos, q_yaw, 0.05, 0.05])
        self.R_odom = np.diag([r_odom, r_odom])
        self.R_imu = np.array([[r_imu]])

    def predict(self, dt):
        if not math.isfinite(dt) or not 0 < dt <= 0.5:
            raise ValueError('Prediction dt must be in (0, 0.5]')
        theta, v, omega = self.x[2:, 0]
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v*math.sin(theta)*dt, math.cos(theta)*dt
        F[1, 2], F[1, 3] = v*math.cos(theta)*dt, math.sin(theta)*dt
        F[2, 4] = dt
        self.x[0, 0] += v*math.cos(theta)*dt
        self.x[1, 0] += v*math.sin(theta)*dt
        self.x[2, 0] = normalize_angle(theta+omega*dt)
        self.P = F@self.P@F.T + self.Q*dt

    def update(self, values, indices, covariance):
        z = np.asarray(values, dtype=float).reshape(-1, 1)
        if not np.isfinite(z).all():
            return False
        H = np.eye(5)[list(indices)]
        S = H@self.P@H.T + covariance
        K = np.linalg.solve(S, H@self.P).T
        self.x += K@(z-H@self.x)
        self.x[2, 0] = normalize_angle(self.x[2, 0])
        A = np.eye(5)-K@H
        self.P = A@self.P@A.T + K@covariance@K.T
        self.P = (self.P+self.P.T)*0.5
        return True


class Tracker:
    def __init__(self, kx=1.4, ky=2.8, ktheta=2.0, ki_v=0.3,
                 integral_limit=0.4, v_max=1.5, omega_max=2.0, accel_max=2.5):
        values = locals().copy(); values.pop('self')
        if not all(math.isfinite(v) and v >= 0 for v in values.values()):
            raise ValueError('Controller gains and limits must be finite and nonnegative')
        self.__dict__.update(values)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_v = 0.0

    def step(self, pose, t, dt):
        if not all(math.isfinite(v) for v in (*pose, t, dt)) or not 1e-5 < dt <= 0.2:
            self.reset()
            return 0.0, 0.0
        xr, yr, theta, vr, wr = reference(t)
        x, y, yaw = pose
        dx, dy = xr-x, yr-y
        ex = math.cos(yaw)*dx+math.sin(yaw)*dy
        ey = -math.sin(yaw)*dx+math.cos(yaw)*dy
        et = normalize_angle(theta-yaw)
        self.integral = float(np.clip(self.integral+ex*dt, -self.integral_limit, self.integral_limit))
        raw = vr*math.cos(et)+self.kx*ex+self.ki_v*self.integral
        v = float(np.clip(raw, self.previous_v-self.accel_max*dt, self.previous_v+self.accel_max*dt))
        v = float(np.clip(v, -self.v_max, self.v_max))
        omega = float(np.clip(wr+vr*(self.ky*ey+self.ktheta*math.sin(et)), -self.omega_max, self.omega_max))
        self.previous_v = v
        return v, omega
