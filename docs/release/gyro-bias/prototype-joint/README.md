# Rejected full joint EKF prototype — evidence retained

This was the first six-state `gyro = omega + bias` implementation, with full pose Kalman gain. Its sources, 110 paired 240-second trials, two 240-second ROS runs, 36-result automated suite and twelve ROS scenario checks are retained here. The 120-trial 30-second matrix for this prototype remains at `docs/experiments_bias_v2/`; the original five-state matrix remains at `docs/experiments/`.

The prototype reduced nominal last-30-second true tracking RMSE from 2.3168 m to 0.2232 m (five paired seeds), but sustained-slip true RMSE increased from 3.0323 m to 13.9426 m, with estimated RMSE 8.3229 m. Although stream/finite/stop/recovery checks passed, this was not acceptable trajectory tracking. No unfavorable runs were discarded.

It was replaced by a consider-style measurement update: rates/bias learn while the pose mean is propagated only, retaining full covariance/cross-covariance with the Joseph update. This policy is applied uniformly rather than depending on simulator scenario or ground truth. Final results live outside this prototype directory; this directory must not be presented as the final estimator's measurements.
