# Paired multi-minute gyro-bias validation

110 trials; 240 simulated seconds each; paired seeds [11, 23, 42, 71, 101].

Ground truth is evaluation-only. Default injected bias stays +0.005 rad/s; signed-bias stress cases use -0.02/+0.03 rad/s.
No per-scenario tuning. Original artifacts remain untouched. All failures and unfavorable pose errors are retained.

| Profile | Bias estimation | Last 30 s TRUE RMSE mean ± SD [m] | Last 30 s ESTIMATED RMSE mean ± SD [m] | Final bias mean [rad/s] |
|---|---|---:|---:|---:|
| nominal | False | 2.3168 ± 0.0277 | 0.0223 ± 0.0005 | 0.000000 |
| nominal | True | 0.2232 ± 0.0670 | 0.0225 ± 0.0006 | 0.004713 |
| load | False | 2.3165 ± 0.0277 | 0.0223 ± 0.0005 | 0.000000 |
| load | True | 0.2221 ± 0.0667 | 0.0225 ± 0.0006 | 0.004713 |
| slip_transient | False | 2.5417 ± 0.0254 | 0.0223 ± 0.0005 | 0.000000 |
| slip_transient | True | 1.2325 ± 0.0466 | 0.0227 ± 0.0005 | 0.003899 |
| slip_sustained | False | 3.0323 ± 0.0351 | 0.1272 ± 0.0002 | 0.000000 |
| slip_sustained | True | 13.9426 ± 0.5462 | 8.3229 ± 4.1441 | -0.009023 |
| imu_dropout | False | 4.0641 ± 0.0436 | 0.0223 ± 0.0005 | 0.000000 |
| imu_dropout | True | 3.1255 ± 0.0962 | 0.0227 ± 0.0006 | 0.003711 |
| odom_dropout | False | 3.9731 ± 0.0488 | 0.0223 ± 0.0005 | 0.000000 |
| odom_dropout | True | 2.6070 ± 0.0991 | 0.0225 ± 0.0006 | 0.004742 |
| odom_delay | False | 3.9731 ± 0.0488 | 0.0223 ± 0.0005 | 0.000000 |
| odom_delay | True | 2.6070 ± 0.0991 | 0.0225 ± 0.0006 | 0.004742 |
| nonfinite | False | 3.9731 ± 0.0488 | 0.0223 ± 0.0005 | 0.000000 |
| nonfinite | True | 2.6070 ± 0.0991 | 0.0225 ± 0.0006 | 0.004742 |
| command_dropout | False | 2.3192 ± 0.0295 | 0.0223 ± 0.0005 | 0.000000 |
| command_dropout | True | 0.2293 ± 0.0667 | 0.0225 ± 0.0006 | 0.004713 |
| negative_bias | False | 3.7823 ± 0.0211 | 0.0242 ± 0.0004 | 0.000000 |
| negative_bias | True | 0.2202 ± 0.0654 | 0.0225 ± 0.0006 | -0.020277 |
| positive_bias | False | 0.7122 ± 0.0183 | 0.0260 ± 0.0004 | 0.000000 |
| positive_bias | True | 0.2264 ± 0.0686 | 0.0225 ± 0.0006 | 0.029704 |

Numerical finite/command/fault-stop/recovery checks: 110/110 trials passed.
This check count is not a declaration of acceptable tracking in slip/fault cases. See true-pose errors above.
Wheel slip and wheel scale/yaw bias are not independently separable from gyro bias with these sensors.
No absolute pose correction is provided. Estimated tracking error can remain small while true-pose tracking is poor.

![True/estimated error and learned bias](drift.png)
