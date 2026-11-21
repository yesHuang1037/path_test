#!/usr/bin/env python3
"""End-to-end checks using small synthetic maps; no display or GUI calls."""
import argparse
import csv
import importlib.util
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
args = argparse.ArgumentParser()
args.add_argument('--executable', type=Path, default=ROOT / 'build/play')
OPTIONS, REMAINING = args.parse_known_args()
sys.argv = [sys.argv[0]] + REMAINING
spec = importlib.util.spec_from_file_location('benchmark_script', ROOT / 'scripts/run_benchmark.py')
MODULE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(MODULE)


def load(path):
    return json.loads(path.read_text())


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='tdt-benchmark-')
        self.root = Path(self.temp.name)
        self.environment = dict(os.environ, DISPLAY='', QT_QPA_PLATFORM='offscreen', PYTHONDONTWRITEBYTECODE='1')

    def tearDown(self):
        self.temp.cleanup()

    def image(self, name='map.pgm', width=60, height=40, wall=False, occupied=False):
        data = bytearray([0 if occupied else 255] * (width*height))
        if wall:
            for y in range(height):
                data[y*width+width//2] = 0
        path = self.root / name
        path.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + data)
        return path

    def run_script(self, output, *options, expected=0):
        command = [sys.executable, '-B', str(ROOT / 'scripts/run_benchmark.py'),
                   '--executable', str(OPTIONS.executable.resolve()), '--output-dir', str(output)] + list(map(str, options))
        result = subprocess.run(command, cwd='/tmp', env=self.environment, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result

    def direct(self, *options):
        return subprocess.run([str(OPTIONS.executable.resolve())] + list(map(str, options)),
                              cwd='/tmp', env=self.environment, capture_output=True, text=True, timeout=60)

    def test_seed_map_report_and_historical_rerun(self):
        image = self.image()
        original, duplicate, rerun = [self.root / x for x in ('original', 'duplicate', 'rerun')]
        result = self.run_script(original, '--map', image, '--resolution', '.1', '--count', 8, '--seed', 314159)
        self.run_script(duplicate, '--map', image, '--resolution', '.1', '--count', 8, '--seed', 314159)
        inputs = load(original / 'inputs.json')
        self.assertEqual(inputs['cases'], load(duplicate / 'inputs.json')['cases'])
        self.assertEqual(len(inputs['cases']), 8)
        for case in inputs['cases']:
            self.assertTrue(0 <= case['start_x'] < 6)
            self.assertTrue(0 <= case['start_y'] < 4)
            self.assertNotEqual((case['start_x'], case['start_y']), (case['end_x'], case['end_y']))
        rows = MODULE.read_rows(original)
        self.assertEqual(len(rows), 8)
        self.assertEqual(sum(row['passed'] == '1' for row in rows), 8)
        for row in rows:
            self.assertGreater(float(row['total_ms']), 0)
            self.assertGreaterEqual(float(row['total_ms']), sum(float(row[s]) for s in MODULE.STAGES[:-1] if row[s]))
        self.assertIn('total_ms=', result.stdout)
        with (original / 'summary.csv').open() as stream:
            summary = {row['group']: row for row in csv.DictReader(stream)}
        self.assertEqual(summary['random']['pass_rate_pct'], '100.0')
        with (original / 'timings.csv').open() as stream:
            timing = next(r for r in csv.DictReader(stream) if r['group']=='random' and r['outcome']=='all' and r['stage']=='total_ms')
        self.assertAlmostEqual(float(timing['sum_ms']), sum(float(r['total_ms']) for r in rows))
        self.run_script(rerun, '--rerun', original, '--case-id', 3, '--headless')
        self.assertEqual(load(rerun / 'inputs.json')['cases'], inputs['cases'])
        self.assertEqual([r['case_id'] for r in MODULE.read_rows(rerun)], ['3'])
        self.assertEqual(load(rerun / 'manifest.json')['seed'], 314159)
        a, b = load(original / 'cases/3.json'), load(rerun / 'cases/3.json')
        self.assertEqual(a['path'], b['path'])
        self.assertEqual(a['reason'], b['reason'])
        for name in ('run.log', 'summary.md', 'paths.csv', 'corridors.csv', 'cases.csv', 'failures.csv'):
            self.assertTrue((original / name).is_file())
        # Historical snapshot checksum is enforced before executing a rerun.
        (original / 'map.png').write_bytes(b'corrupted')
        self.run_script(self.root / 'corrupt', '--rerun', original, '--headless', expected=2)

    def test_unreachable_cases_count_as_failure_and_skip_backend(self):
        output = self.root / 'wall'
        self.run_script(output, '--map', self.image(wall=True), '--count', 20, '--seed', 42)
        rows = MODULE.read_rows(output)
        failures = [r for r in rows if r['reason']=='frontend_failure']
        self.assertTrue(failures)
        for row in failures:
            self.assertEqual(row['backend_attempted'], '0')
            self.assertEqual(row['optimize_ms'], '')
            self.assertGreater(float(row['total_ms']), 0)
        with (output / 'timings.csv').open() as stream:
            records = list(csv.DictReader(stream))
        search = next(r for r in records if r['group']=='random' and r['outcome']=='all' and r['stage']=='search_ms')
        optimize = next(r for r in records if r['group']=='random' and r['outcome']=='all' and r['stage']=='optimize_ms')
        self.assertEqual(int(search['count']), 20)
        self.assertEqual(int(optimize['count']), 20-len(failures))
        rerun = self.root / 'failures'
        self.run_script(rerun, '--rerun', output, '--headless', '--failed-only')
        self.assertEqual(len(MODULE.read_rows(rerun)), len([r for r in rows if r['passed']=='0']))

    def test_time_seed_and_bad_inputs(self):
        import time
        start = int(time.time())
        output = self.root / 'timeseed'
        self.run_script(output, '--map', self.image(width=30, height=20), '--count', 1)
        seed = load(output / 'manifest.json')['seed']
        self.assertTrue(start <= seed <= int(time.time()))
        for options in (('--resolution', 'nan'), ('--seed', '-1'), ('--count', '-1'),
                        ('--mode', 'unknown'), ('--resolution', '0'), ('--map', '/missing/map.png')):
            self.assertEqual(self.direct(*options).returncode, 2)
        bad = self.root / 'occupied'
        self.run_script(bad, '--map', self.image('empty.pgm', occupied=True), '--count', 1, expected=2)
        self.assertEqual(load(bad / 'manifest.json')['status'], 'failed')
        self.assertEqual(MODULE.read_rows(bad), [])

    def test_saved_replay_selection_and_summary_without_completed_run(self):
        directory = self.root / 'partial'
        directory.mkdir()
        manifest = {'status': 'interrupted'}
        rows = MODULE.summarize(directory, manifest)
        self.assertEqual(rows, [])
        self.assertIn('interrupted', (directory / 'summary.md').read_text())
        inputs = {'cases': [{'id': 1}, {'id': 2}, {'id': 3}]}
        selected = MODULE.selected_cases(inputs, argparse.Namespace(failed_only=True, case_id=None, from_case=2),
                                        [{'case_id': '1', 'passed': '0'}, {'case_id': '3', 'passed': '0'}])
        self.assertEqual(selected, [{'id': 3}])

    def test_interruption_preserves_partial_results(self):
        output = self.root / 'interrupted'
        command = [sys.executable, '-B', str(ROOT/'scripts/run_benchmark.py'),
                   '--executable', str(OPTIONS.executable.resolve()), '--map', str(self.image()),
                   '--count', '2000', '--seed', '42', '--output-dir', str(output)]
        process = subprocess.Popen(command, env=self.environment, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic()+20
            while time.monotonic()<deadline and process.poll() is None:
                if len(MODULE.read_rows(output)) >= 2:
                    break
                time.sleep(.01)
            self.assertIsNone(process.poll(), 'Benchmark finished before interruption test')
            process.send_signal(signal.SIGINT)
            _, errors = process.communicate(timeout=15)
            self.assertEqual(process.returncode, 130, errors)
            manifest = load(output/'manifest.json')
            self.assertEqual(manifest['status'], 'interrupted')
            self.assertGreater(manifest['completed_cases'], 0)
            self.assertLess(manifest['completed_cases'], 2000)
            self.assertTrue((output/'summary.md').is_file())
            for row in MODULE.read_rows(output):
                self.assertEqual(load(output/'cases'/f"{row['case_id']}.json")['case_id'], int(row['case_id']))
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


if __name__ == '__main__':
    unittest.main()
