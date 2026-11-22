# Offline slip-aware AMR twin audit

## Implemented scope

AMRDigitalTwin is a ROS-independent offline mirror using wheel linear/yaw rates
and raw gyro yaw rate. It estimates a gyro baseline and applies a traction factor
to wheel linear velocity during an alarm. The simulator assumes a shared
linear/yaw traction factor; real slip need not satisfy that assumption.
The twin is not connected to the live estimator, controller, or WebSocket bridge.
Ground truth is used only for evaluation.

## Verified repairs

1. Duplicate/out-of-order timestamps return valid=False before any internal
   mutation. A clock restart requires an explicit reset.
2. Replay truth is captured at sensor acquisition. EKF pose and reference are
   aligned to that time, replacing a comparison to a row 10 ms later.

## Archived exploratory evidence

The archive contains a corrected nominal/slip replay at seed 1101, plus 40
full-twin and 40 no-traction ablation trials across seeds 1201-1210. Each trial
uses 30 simulated seconds, one figure-eight trajectory/controller, and slip
window [12,16). Seeds vary sensor noise, not independent trajectories.
These are mean localization RMSE values, not closed-loop tracking gains.

| Slip variant | EKF RMSE [m] | Full twin RMSE [m] | No-traction twin RMSE [m] |
|---|---:|---:|---:|
| matched | 1.0499 | 0.4552 | 0.8439 |
| linear-only | 0.8649 | 0.8339 | 0.8339 |
| yaw-only | 0.4103 | 0.4191 | 0.0659 |
| half-linear | 0.6774 | 0.1252 | 0.4269 |

Traction application helps in the tested matched and half-linear variants,
does nothing in the tested linear-only variants, and harms all ten yaw-only
seeds relative to the otherwise identical no-traction twin. Its small
linear-only advantage over EKF is not attributable to traction application.
There is no statistical-significance, independent held-out, or hardware claim.

The detector observes yaw disagreement. It cannot identify pure longitudinal
slip or separate all wheel-scale, gyro-bias, and slip changes from these signals.
The twin assumes linear/yaw traction coupling rather than measuring it.
Do not infer reliable linear correction from a yaw alarm alone.

## Reproduce from the repository root

Run with NumPy installed and a fresh output directory:

    OPENBLAS_NUM_THREADS=1 python3 scripts/reproduce_twin_audit.py --output-dir /tmp/amr-twin-audit-new --verify-archive

The default runs ten archived seeds: 80 twin evaluations and two matched-model
precheck runs. A quick verification used by CI runs two seeds:

    OPENBLAS_NUM_THREADS=1 python3 scripts/reproduce_twin_audit.py --output-dir /tmp/amr-twin-audit-quick --seeds 1201 1202 --verify-archive

The runner writes fresh JSON outside the repository and verifies per-seed
metrics against the archive at absolute tolerance 1e-12. Source-version checks
reject comparisons to evidence produced by different core versions.
The ablation changes only linear traction application in an isolated in-memory class.
The controller and sensor streams are unaffected by either twin.

## Evidence and automation

The [frozen archive manifest](research/twin-audit-2026-11-22/manifest.json)
identifies results and source snapshots. Its original summary states that
standalone runners were absent when archived; the runner above is added here.
Earlier numerical outputs remain preserved.
CTest registers slip_detector, digital_twin, and twin_replay, including
timestamp rejection and replay alignment regression cases. The existing ROS CI
also builds/tests the original stack and performs the quick archive reproduction.

This is a tested offline component with measured model-dependence limits.
Live sensing integration, uncertainty-aware correction, wider trajectories/faults,
and hardware validation remain separate milestones.
