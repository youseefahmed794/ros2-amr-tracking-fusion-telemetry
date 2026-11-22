#!/usr/bin/env python3
"""Reproduce the offline twin audit; no ROS or project-result writes.

The full twin and a one-line in-memory ablation receive the same simulated
sensor streams. Only linear traction application differs. Ground truth is
used for scoring, not as input to the twin or controller.
"""
import argparse
import hashlib
import importlib.util
import json
import platform
import sys
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
import slip_model_mismatch_check as mismatch


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(output, seeds, verify_archive=False):
    """Run paired full/no-traction mirrors for four fixed slip variants."""
    output = Path(output)
    if output.exists():
        raise FileExistsError('Use a new output directory; existing evidence is preserved')
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('Provide at least one seed, without duplicates')
    spec = importlib.util.spec_from_file_location(
        'audited_twin_replay', ROOT / 'scripts/digital_twin_replay.py',
    )
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)
    sim = replay.sim
    if sim is not mismatch.sim:
        raise RuntimeError('Replay and mismatch helpers imported different simulators')
    if replay.DURATION != 30.0 or tuple(replay.FAULT) != (12.0, 16.0):
        raise RuntimeError('The audited protocol requires 30 s and fault interval [12,16)')

    source_paths = [
        'ros2_amr_nmpc_telemetry/core.py',
        'ros2_amr_nmpc_telemetry/simulation.py',
        'ros2_amr_nmpc_telemetry/slip_detector.py',
        'ros2_amr_nmpc_telemetry/digital_twin.py',
        'scripts/digital_twin_replay.py',
        'scripts/slip_model_mismatch_check.py',
        'scripts/slip_compensation_experiment.py',
        'scripts/reproduce_twin_audit.py',
    ]
    source_hashes = {name: sha(ROOT / name) for name in source_paths}
    archive = ROOT / 'docs/research/twin-audit-2026-11-22'
    full_reference = {}
    ablation_reference = {}
    evidence_hashes = {}
    if verify_archive:
        for filename in ('twin_model_mismatch_evidence.json', 'traction_ablation_evidence.json'):
            evidence_hashes[filename] = sha(archive / filename)
        full = json.loads((archive / 'twin_model_mismatch_evidence.json').read_text())
        ablation = json.loads((archive / 'traction_ablation_evidence.json').read_text())
        if ablation['previous_evidence_sha256'] != evidence_hashes['twin_model_mismatch_evidence.json']:
            raise ValueError('Archived ablation provenance does not match the full-twin evidence')
        for name, expected in full['source_sha256'].items():
            if source_hashes.get(name) != expected:
                raise ValueError('Archive source version differs: ' + name)
        if ablation['source_sha256'] != full['source_sha256']:
            raise ValueError('Archived source baselines differ')
        full_reference = {(r['variant'], r['seed']): r for r in full['trials']}
        ablation_reference = {(r['variant'], r['seed']): r for r in ablation['trials']}
        required = {(name, seed) for name, _, _ in mismatch.VARIANTS for seed in seeds}
        if not required.issubset(full_reference) or not required.issubset(ablation_reference):
            raise ValueError('Requested seeds are not all present in the archive')

    # Keep all detector, bias, yaw integration and timing code identical.
    original_source = (ROOT / 'ros2_amr_nmpc_telemetry/digital_twin.py').read_text()
    before = '            v = self.traction * wheel_v\n'
    after = '            v = wheel_v\n'
    if original_source.count(before) != 1:
        raise ValueError('The reviewed ablation anchor must occur exactly once')
    ablated_source = original_source.replace(before, after, 1)
    module = types.ModuleType('ros2_amr_nmpc_telemetry._audit_no_traction')
    module.__package__ = 'ros2_amr_nmpc_telemetry'
    exec(compile(ablated_source, '<audit-no-traction>', 'exec'), module.__dict__)

    original_step = sim.DifferentialDrivePlant.step
    original_sensors = sim.DifferentialDrivePlant.sensors
    original_twin = replay.AMRDigitalTwin
    original_seed = replay.SEED
    records = []
    max_reference_difference = 0.0
    try:
        reference, _ = sim.run_trial(
            'lyapunov_integral', 'slip', seeds[0], replay.DURATION,
            fault_windows=(replay.FAULT,),
        )
        sim.DifferentialDrivePlant.step = mismatch.make_step(
            mismatch.VARIANTS[0][1], mismatch.VARIANTS[0][2],
        )
        copied, _ = sim.run_trial(
            'lyapunov_integral', 'slip', seeds[0], replay.DURATION,
            fault_windows=(replay.FAULT,),
        )
        np.testing.assert_allclose(copied, reference, rtol=0.0, atol=1e-12)
        print('MATCHED_MODEL_EQUIVALENCE_CONFIRMED', flush=True)
        for variant, v_factor, w_factor in mismatch.VARIANTS:
            sim.DifferentialDrivePlant.step = mismatch.make_step(v_factor, w_factor)
            for seed in seeds:
                for mode, twin_class in (
                    ('full', original_twin), ('no_traction', module.AMRDigitalTwin),
                ):
                    replay.SEED = seed
                    replay.AMRDigitalTwin = twin_class
                    rows, _ = replay.replay('slip')
                    truth = np.asarray([r['true'] for r in rows])
                    ekf = np.asarray([r['ekf'] for r in rows])
                    twin = np.asarray([r['twin'] for r in rows])

                    def rmse(estimate):
                        return float(np.sqrt(np.mean(np.sum((estimate - truth) ** 2, axis=1))))

                    item = {
                        'variant': variant, 'seed': seed, 'mode': mode,
                        'samples': len(rows),
                        'ekf_localization_rmse_m': rmse(ekf),
                        'twin_localization_rmse_m': rmse(twin),
                    }
                    if verify_archive:
                        prior = full_reference[(variant, seed)]
                        target = (
                            prior['twin_localization_rmse_m'] if mode == 'full'
                            else ablation_reference[(variant, seed)]['no_traction_twin_localization_rmse_m']
                        )
                        observed = [item['ekf_localization_rmse_m'], item['twin_localization_rmse_m']]
                        expected = [prior['ekf_localization_rmse_m'], target]
                        if len(rows) != prior['samples']:
                            raise ValueError('Replay sample count differs from archived evidence')
                        np.testing.assert_allclose(observed, expected, rtol=0.0, atol=1e-12)
                        max_reference_difference = max(
                            max_reference_difference,
                            float(np.max(np.abs(np.asarray(observed) - expected))),
                        )
                    records.append(item)
            print('COMPLETED_VARIANT:', variant, 'SEEDS:', seeds, flush=True)
    finally:
        sim.DifferentialDrivePlant.step = original_step
        sim.DifferentialDrivePlant.sensors = original_sensors
        replay.AMRDigitalTwin = original_twin
        replay.SEED = original_seed

    summaries = []
    for variant, _, _ in mismatch.VARIANTS:
        full = [r for r in records if r['variant'] == variant and r['mode'] == 'full']
        ablated = [r for r in records if r['variant'] == variant and r['mode'] == 'no_traction']
        differences = np.asarray([
            f['twin_localization_rmse_m'] - a['twin_localization_rmse_m']
            for f, a in zip(full, ablated)
        ])
        summary = {
            'variant': variant, 'seeds': seeds,
            'ekf_mean_rmse_m': float(np.mean([r['ekf_localization_rmse_m'] for r in full])),
            'full_twin_mean_rmse_m': float(np.mean([r['twin_localization_rmse_m'] for r in full])),
            'no_traction_twin_mean_rmse_m': float(np.mean([r['twin_localization_rmse_m'] for r in ablated])),
            'mean_paired_full_minus_no_traction_rmse_m': float(differences.mean()),
            'seeds_traction_lowers_rmse': int(np.sum(differences < 0)),
            'seeds_traction_raises_rmse': int(np.sum(differences > 0)),
            'seeds_equal_rmse': int(np.sum(differences == 0)),
        }
        summaries.append(summary)
        print(json.dumps(summary, allow_nan=False), flush=True)

    payload = {
        'scope': 'Exploratory offline twin localization; one trajectory/controller; simulation only.',
        'note': 'The twin does not control the plant. Ground truth is used only for scoring.',
        'seeds': seeds, 'duration_s': replay.DURATION,
        'fault_window_s': list(replay.FAULT), 'trial_count': len(records),
        'matched_model_precheck_runs': 2,
        'archive_verified': verify_archive,
        'archive_comparison_atol': 1e-12 if verify_archive else None,
        'max_archive_metric_difference': max_reference_difference if verify_archive else None,
        'python': platform.python_version(), 'numpy': np.__version__,
        'source_sha256': source_hashes, 'reference_evidence_sha256': evidence_hashes,
        'in_memory_ablation_sha256': hashlib.sha256(ablated_source.encode()).hexdigest(),
        'summaries': summaries, 'trials': records,
    }
    output.mkdir(parents=True, exist_ok=False)
    path = output / 'twin_audit_reproduction.json'
    with path.open('x', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, allow_nan=False)
    print('EVIDENCE_PATH:', path, flush=True)
    print('EVIDENCE_SHA256:', sha(path), flush=True)
    print('ARCHIVE_VERIFIED:', verify_archive, flush=True)
    print('AUDIT_TRIAL_COUNT:', len(records), flush=True)
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(1201, 1211)))
    parser.add_argument('--verify-archive', action='store_true')
    args = parser.parse_args()
    run(args.output_dir, args.seeds, args.verify_archive)


if __name__ == '__main__':
    main()
