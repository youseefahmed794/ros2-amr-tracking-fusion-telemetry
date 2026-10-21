"""Seeded differential-drive dynamics, independent of ROS and rendering.

Models wheel actuator lag/acceleration, encoder noise, IMU noise/bias,
wheel-ground traction loss, and an independent base command watchdog.
"""
from dataclasses import dataclass
import math
import numpy as np
from .core import normalize_angle, reference, Tracker, VelocityEKF


@dataclass(frozen=True)
class PlantConfig:
    wheel_radius: float = 0.1
    wheel_separation: float = 0.4
    motor_time_constant: float = 0.12
    wheel_accel_limit: float = 35.0
    command_timeout: float = 0.2
    encoder_std: float = 0.03
    imu_std: float = 0.01
    imu_bias: float = 0.005


class DifferentialDrivePlant:
    def __init__(self, seed=42, config=None):
        self.config = config or PlantConfig()
        if any(not math.isfinite(v) or v <= 0 for v in (
                self.config.wheel_radius, self.config.wheel_separation,
                self.config.motor_time_constant, self.config.wheel_accel_limit,
                self.config.command_timeout)):
            raise ValueError('Plant dimensions, dynamics and timeout must be positive')
        self.rng = np.random.default_rng(seed)
        self.pose = np.zeros(3)
        self.wheels = np.zeros(2)
        self.wheel_position = np.zeros(2)
        self.target = np.zeros(2)
        self.last_command = -math.inf
        self.velocity = np.zeros(2)
        self.watchdog = True

    def command(self, v, w, t):
        if not all(math.isfinite(value) for value in (v, w, t)):
            return False
        c = self.config
        self.target = np.array([v-w*c.wheel_separation/2,
                                v+w*c.wheel_separation/2])/c.wheel_radius
        self.last_command = t
        return True

    def step(self, t, dt, traction=1.0, load=1.0):
        if not 0 < dt <= 0.1 or not 0 <= traction <= 1 or not 0 < load <= 1:
            raise ValueError('Invalid plant integration step or disturbance')
        c = self.config
        age = t-self.last_command
        self.watchdog = not 0 <= age < c.command_timeout
        target = np.zeros(2) if self.watchdog else self.target*load
        delta = (target-self.wheels)*(1-math.exp(-dt/c.motor_time_constant))
        self.wheels += np.clip(delta, -c.wheel_accel_limit*dt, c.wheel_accel_limit*dt)
        self.wheel_position += self.wheels*dt
        left, right = self.wheels*c.wheel_radius
        v = traction*(left+right)/2
        w = traction*(right-left)/c.wheel_separation
        # Midpoint integration; the estimator uses its own process model.
        self.pose[0] += v*math.cos(self.pose[2]+w*dt/2)*dt
        self.pose[1] += v*math.sin(self.pose[2]+w*dt/2)*dt
        self.pose[2] = normalize_angle(self.pose[2]+w*dt)
        self.velocity[:] = v, w

    def sensors(self):
        c = self.config
        left, right = self.wheels*c.wheel_radius
        # Draw a fixed number of samples each tick for paired controller trials.
        wheel = np.array([(left+right)/2, (right-left)/c.wheel_separation])
        wheel += self.rng.normal(0, c.encoder_std, 2)
        imu = self.velocity[1]+c.imu_bias+self.rng.normal(0, c.imu_std)
        return wheel, imu


class PurePursuit:
    """Time-preview baseline on the same reference, speed and output limits."""
    def __init__(self):
        self.previous_v = 0.0

    def reset(self):
        self.previous_v = 0.0

    def step(self, pose, t, dt):
        target = reference(t+0.6)
        x, y, yaw = pose
        dx, dy = target[0]-x, target[1]-y
        lateral = -math.sin(yaw)*dx+math.cos(yaw)*dy
        distance_squared = max(dx*dx+dy*dy, 0.04)
        speed = reference(t)[3]
        v = float(np.clip(speed, self.previous_v-2.5*dt, self.previous_v+2.5*dt))
        v = float(np.clip(v, -1.5, 1.5))
        w = float(np.clip(2*v*lateral/distance_squared, -2, 2))
        self.previous_v = v
        return v, w


CONTROLLERS = ('lyapunov_integral', 'lyapunov_no_integral', 'pure_pursuit')
SCENARIOS = ('nominal', 'load', 'slip', 'imu_dropout', 'odom_dropout',
             'odom_delay', 'nonfinite', 'command_dropout')


def run_trial(controller='lyapunov_integral', scenario='nominal', seed=42, duration=30.0,
              estimate_gyro_bias=True, fault_windows=((12.0, 16.0),), plant_config=None):
    if controller not in CONTROLLERS or scenario not in SCENARIOS:
        raise ValueError('Unknown controller or scenario')
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('Duration must be finite and positive')
    if any(not (math.isfinite(start) and math.isfinite(end) and 0 <= start < end)
           for start, end in fault_windows):
        raise ValueError('Fault windows must be finite ordered intervals')
    tracker = (PurePursuit() if controller == 'pure_pursuit'
               else Tracker(ki_v=0.3 if controller == 'lyapunov_integral' else 0.0))
    plant, ekf = DifferentialDrivePlant(seed, plant_config), VelocityEKF(estimate_gyro_bias=estimate_gyro_bias)
    dt = 0.01
    last_odom = last_imu = -math.inf
    filtered_stamp = -math.inf
    delayed = []
    rows = []
    rejected = 0
    first_stop = None
    for k in range(int(duration/dt)):
        t = k*dt
        fault = any(start <= t < end for start, end in fault_windows)
        if k % 2 == 0:
            wheel, imu = plant.sensors()
            if scenario == 'nonfinite' and fault:
                wheel[:] = math.nan
            if not (scenario == 'imu_dropout' and fault):
                if ekf.update_imu(imu):
                    last_imu = t
            if not (scenario == 'odom_dropout' and fault):
                delay = 0.8 if scenario == 'odom_delay' and fault else 0
                delayed.append((t+delay, t, wheel.copy()))
            ready = [entry for entry in delayed if entry[0] <= t+1e-9]
            delayed = [entry for entry in delayed if entry[0] > t+1e-9]
            for _, stamp, wheel in ready:
                if not 0 <= t-stamp <= 0.25 or stamp <= last_odom:
                    rejected += 1
                    continue
                if ekf.update(wheel, (3, 4), ekf.R_odom):
                    last_odom = stamp
                else:
                    rejected += 1
        if t-last_odom < 0.5 and t-last_imu < 0.5:
            ekf.predict(dt)
            if k % 2 == 0:
                filtered_stamp = t
        if t-filtered_stamp < 0.5:
            command = tracker.step(ekf.x[:3, 0], t, dt)
            stopped = False
        else:
            tracker.reset()
            command = (0.0, 0.0)
            stopped = True
        if not (scenario == 'command_dropout' and fault):
            plant.command(*command, t)
        plant.step(t, dt, traction=0.65 if scenario == 'slip' and fault else 1,
                   load=0.7 if scenario == 'load' and fault else 1)
        if fault and first_stop is None and (stopped or plant.watchdog):
            first_stop = t-next(start for start, end in fault_windows if start <= t < end)
        ref = reference(t+dt)
        rows.append([t+dt, *plant.pose, *ekf.x[:3, 0], ref[0], ref[1],
                     *command, *plant.velocity, float(stopped), float(plant.watchdog), float(ekf.x[5, 0]), float(ekf.P[5, 5])])
    data = np.asarray(rows)
    error = np.linalg.norm(data[:, 1:3]-data[:, 7:9], axis=1)
    estimated_error = np.linalg.norm(data[:, 4:6]-data[:, 7:9], axis=1)
    tail = data[:, 0] > max(0.0, duration-30.0)
    changes = np.diff(data[:, 9:11], axis=0)
    recovery = data[:, 0] >= max((end for start,end in fault_windows), default=0)
    metrics = {
        'controller': controller, 'scenario': scenario, 'seed': seed,
        'duration_sec': duration, 'dt_sec': dt,
        'tracking_rmse_m': float(np.sqrt(np.mean(error**2))),  # historical alias: TRUE pose
        'true_tracking_rmse_m': float(np.sqrt(np.mean(error**2))),
        'estimated_tracking_rmse_m': float(np.sqrt(np.mean(estimated_error**2))),
        'last_30s_true_tracking_rmse_m': float(np.sqrt(np.mean(error[tail]**2))),
        'last_30s_estimated_tracking_rmse_m': float(np.sqrt(np.mean(estimated_error[tail]**2))),
        'final_heading_error_est_minus_true_rad': normalize_angle(data[-1,6]-data[-1,3]),
        'final_gyro_bias_rad_s': float(data[-1,15]),
        'final_gyro_bias_variance': float(data[-1,16]),
        'injected_gyro_bias_rad_s': plant.config.imu_bias,
        'estimate_gyro_bias': estimate_gyro_bias,
        'max_error_m': float(error.max()),
        'post_fault_rmse_m': float(np.sqrt(np.mean(error[recovery]**2))) if recovery.any() else None,
        'localization_rmse_m': float(np.sqrt(np.mean(np.sum((data[:, 1:3]-data[:, 4:6])**2, axis=1)))),
        'linear_command_total_variation': float(np.abs(changes[:, 0]).sum()),
        'angular_command_total_variation': float(np.abs(changes[:, 1]).sum()),
        'stop_detection_sec': first_stop, 'rejected_measurements': rejected,
        'peak_linear_command_m_s': float(np.abs(data[:, 9]).max()),
        'peak_angular_command_rad_s': float(np.abs(data[:, 10]).max()),
    }
    return data, metrics
