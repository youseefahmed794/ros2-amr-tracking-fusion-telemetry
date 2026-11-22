# Yaw-rate slip detector (detection only)

Status: simulation results only. Nothing here has been run on hardware or inside the ROS nodes, and the detector is not connected to the estimator or the controller.

## What it does

`ros2_amr_nmpc_telemetry/slip_detector.py` contains the ROS-independent class `YawRateSlipDetector`. Each update receives the wheel yaw rate and the raw gyro yaw rate in rad/s. It computes the residual r = wheel yaw rate - gyro yaw rate and averages it over a window of 25 samples (0.5 s at 50 Hz). The alarm is active while the absolute difference between that windowed mean and a learned baseline exceeds a threshold.

The baseline is the running mean of the first 100 samples (2 s), then a slow exponential average (alpha = 0.001) that is only updated while the score stays within half of the threshold, so a slip event does not drag the baseline along.

The evaluated configuration is threshold = 0.030 rad/s with confirm_samples = 1. The constructor default (threshold = 0.0196) is NOT the evaluated configuration: it produced 35 false-alarm events in 14.9 minutes of nominal simulation. Pass the threshold explicitly.

## Why it works in this simulator

In `ros2_amr_nmpc_telemetry/simulation.py` the traction factor scales both the linear and the yaw velocity of the plant (`step()`, lines 61-62). The encoder model reports the wheel speeds before that scaling and the gyro model reports the true yaw rate plus bias (`sensors()`, lines 71-75). During a traction loss the residual is therefore about (1 - traction) x wheel yaw rate - gyro bias. With traction = 0.65 the signal is 35% of the wheel yaw rate and it vanishes when the robot is not turning.

## Evaluation protocol

All results come from replaying recorded sensor streams offline with the unchanged simulator (controller lyapunov_integral, 30 s runs unless stated, fault window 12-16 s, traction 0.65 inside the window). Seeds change the noise only; the reference trajectory is fixed.

    Purpose                                   Seeds
    Baseline calibration (nominal)            201-203
    Comparison of 5 candidate configurations  201-203 (30 s), 301-305 (30 s), 401-403 (300 s nominal)
    Pre-registered confirmation               501-510 (30 s), 601-612 (300 s nominal)
    Pre-registered onset-dependence check     701-710

Selection rule, stated after the comparison output had been printed: keep the candidates with zero false-alarm events in the three 300 s nominal runs (seeds 401-403), then take the lowest mean first-alarm latency on calibration seeds 201-203 (0.030/confirm 1: 1.66 s; 0.025/confirm 10: 1.77 s). Seeds 301-305 were printed in the same run and seeds 401-403 were used in the selection, so neither set is independent of the choice. Seeds 501-510, 601-612 and 701-710 were not seen before the choice.

## Results

Confirmation (seeds 501-510 and 601-612; criteria fixed before the run):

    C1 slip detected in at least 9 of 10 seeds          10/10          PASS
    C2 median first-alarm latency <= 2.0 s              1.66 s         PASS
    C3 nominal mean alarm fraction <= 0.005             0.00000        PASS
    C4 load alarm fraction inside fault window <= 0.005 0.00000        PASS
    C5 false-alarm events in 12 x 300 s nominal <= 1    0 in 59.6 min  PASS

The 2.0 s limit in C2 was set after seeing latencies of 1.4-1.8 s in earlier runs, so it is not a blind test. Zero events in 59.6 minutes gives a 95% upper bound of about 0.05 events per minute (rule of three).

Severity sweep (exploratory, seeds 501-510, all 10/10 detected):

    Traction                      0.65   0.80   0.90   0.95
    Median first-alarm latency    1.66   2.07   2.72   3.37  s
    Alarm fraction in window      0.56   0.48   0.31   0.11

Onset dependence (seeds 701-710, 4 s fault window, traction 0.65; criteria O1-O4 all PASS):

    Onset (s)   Median latency (s)   Max latency (s)   Mean |true yaw rate| in first 2 s (rad/s)
    4           0.06                 0.08              0.353
    8           0.08                 0.12              0.599
    12          1.72                 1.88              0.045
    16          0.06                 0.08              0.521
    20          0.12                 0.16              0.527
    24          0.60                 2.10              0.038

Latency is not a constant. It is about 0.1 s at the turn rates measured here (0.35-0.60 rad/s) and up to about 2 s when the loss starts in a slow-turn part of the figure-eight. At onset 24 s two of ten seeds exceeded 2.0 s (2.08 s and 2.10 s); criterion O2 applied to the median only. The mean yaw rate over the first 2 s does not fully predict the latency (onset 24: 0.038 rad/s but median 0.60 s), so the yaw rate right at onset probably matters; this was not measured.

## Limits

1. Simulation only. The simulator scales v and yaw rate by the same traction factor. Real wheel slip does not have to do that, and longitudinal slip with zero yaw rate is invisible to this detector.
2. It is blind while the robot is not turning, so translational slip during straight or slow-turn motion is detected late or not at all.
3. It cannot separate slip from a wheel-scale error or a gyro-bias change. A load change is not flagged here because the simulator applies it to the commanded wheel speeds, which the encoders and the gyro see equally.
4. One reference trajectory (figure-eight), one controller (lyapunov_integral), one 4 s fault length, scenarios nominal, load and slip only. Sensor-fault scenarios (dropouts, delay, non-finite values) were not evaluated.
5. The seeds vary sensor noise only; they are not independent trajectories or fault timings (except in the onset check).
6. Detection only. The estimator's tracking-error estimate stays at about 0.095 m while the true error is about 1.03-1.05 m during slip; no mitigation is claimed.
7. Not tested inside ROS nodes or on hardware.

## Reproduce

Run from the repository root with ROS sourced and the repository root on PYTHONPATH.

    PYTHONPATH="$PWD:$PYTHONPATH" python3 -m pytest test/test_slip_detector.py -q
    python3 scripts/slip_probe.py
    python3 scripts/slip_detector_eval.py
    python3 scripts/slip_detector_confirm.py
    python3 scripts/slip_detector_onset_check.py
