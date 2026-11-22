#!/usr/bin/env python3
"""Frozen, interleaved repetitions of isolated MuJoCo missions (no hardware)."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
import threading
from analyze_transport_run import analyze

# User scope (2026-11-15): omit frustum from routine evaluation. The scene and
# individual runner remain available for a future explicitly requested study.
SHAPES = ['triangle', 'box', 'pentagon', 'sphere', 'ellipsoid', 'cylinder', 'capsule', 'rock', 'sphere_support']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--shapes', nargs='+', choices=SHAPES, default=SHAPES)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--port', type=int, default=11501)
    parser.add_argument('--timeout', type=float, default=600)
    args = parser.parse_args()
    if args.workers not in (1, 2, 3) or args.repeats <= 0:
        parser.error('workers must be 1..3; repeats must be positive')
    args.shapes = list(dict.fromkeys(args.shapes))
    number_of_runs = len(args.shapes) * args.repeats
    if not 1024 <= args.port <= 65536-number_of_runs or 11311 in range(args.port, args.port+number_of_runs):
        parser.error('Use dedicated simulation ports, never 11311')
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    bee = Path(__file__).resolve().parents[1]
    # Freeze the input configurations, source diff and actual tested binaries.
    paths = list((bee / 'config/grasp').glob('*.yaml')) + list((bee/'config/controller').glob('*.yaml'))
    paths += list((bee/'launch').rglob('*.launch')) + list((bee/'launch').rglob('*.xml'))
    paths += list((bee/'src/control').glob('*.cpp')) + list((bee/'include/bee/control').glob('*.h'))
    paths += list((bee/'msg').glob('*.msg')) + list((bee/'srv').glob('*.srv'))
    paths += [Path(__file__), bee / 'test/run_grasp_shape.py', bee / 'test/analyze_transport_run.py']
    paths += list((bee / 'src/cooperative_grasp').glob('*.cpp')) + list((bee / 'include/bee/cooperative_grasp').glob('*'))
    # Hash scripts actually used by the mission, not unrelated offline analyses.
    paths += [bee/'script'/name for name in ['prepare_mujoco_scene.py', 'mujoco_scene_generator.py',
              'grasp_geometry.py', 'grasp_sim_autostart.py', 'mujoco_scene_visualizer.py']]
    devel = bee.parents[3] / 'devel'
    paths += list((devel / 'lib/bee').glob('bee_*'))
    paths += list((devel / 'lib').glob('libbee*.so')) + list((devel / 'lib').glob('libkf*.so'))
    thirdparty = bee.parents[2] / 'aerial_robot_3rdparty/mujoco_ros_control/scripts'
    paths += [thirdparty / 'mujoco_scene_composer.py', thirdparty / 'mujoco_model_generator.py']
    paths += [bee/'mujoco/bee/robot.xml', devel/'lib/mujoco_ros_control/mujoco_ros_control',
              devel/'lib/aerial_robot_base/aerial_robot_base_node']
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
    metadata = dict(started_utc=datetime.now(timezone.utc).isoformat(), conditions=args.shapes,
                    repeats=args.repeats, workers=args.workers, timeout_s=args.timeout,
                    random_seed=61113, hashes=hashes,
                    design='fixed initial conditions, no injected disturbance; scheduling variability only')
    (root / 'protocol.json').write_text(json.dumps(metadata, indent=2)+'\n')
    jobs = [(shape, i+1) for i in range(args.repeats) for shape in args.shapes]
    random.Random(61113).shuffle(jobs)
    lanes = [jobs[i::args.workers] for i in range(args.workers)]
    (root / 'schedule.json').write_text(json.dumps(lanes, indent=2)+'\n')
    lock = threading.Lock()
    all_results = []
    def worker(index):
        for lane_step, (shape, repeat) in enumerate(lanes[index]):
            run = root / ('%s-%02d' % (shape, repeat))
            print('START', index, shape, repeat, flush=True)
            with (root / ('%s-%02d.runner.log' % (shape, repeat))).open('w') as log:
                completed = subprocess.run([sys.executable, str(bee/'test/run_grasp_shape.py'), shape,
                    '--output', str(run), '--port', str(args.port+index+args.workers*lane_step), '--timeout', str(args.timeout)],
                    stdout=log, stderr=subprocess.STDOUT)
            try:
                measured = analyze(run)
                measured['repeat'] = repeat
                measured['runner_returncode'] = completed.returncode
            except Exception as error:
                measured = dict(shape=shape, repeat=repeat, directory=str(run), result='analysis_error',
                                reason=repr(error), runner_returncode=completed.returncode)
            with lock:
                all_results.append(measured)
                (root/'results.json').write_text(json.dumps(all_results, indent=2, allow_nan=False)+'\n')
                print('DONE', len(all_results), '/', len(jobs), shape, repeat,
                      measured['result'], measured.get('reason',''), flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(worker, i) for i in range(args.workers)]):
            future.result()
    changed = [name for name, digest in hashes.items()
               if not Path(name).is_file() or digest != hashlib.sha256(Path(name).read_bytes()).hexdigest()]
    failed = [r['shape'] for r in all_results if r['result'] != 'Complete' or r.get('runner_returncode') != 0]
    (root/'completion.json').write_text(json.dumps(dict(finished_utc=datetime.now(timezone.utc).isoformat(),
        completed=len(all_results), changed_inputs=changed, failed_shapes=failed), indent=2)+'\n')
    return 1 if changed or failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
