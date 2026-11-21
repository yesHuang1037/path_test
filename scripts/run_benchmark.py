#!/usr/bin/env python3
"""Run C++ benchmarks, summarize saved results, or inspect any historical run.

No map configuration files are needed. Use --map and --resolution to change maps.
Python performs orchestration/reporting; algorithm timing is measured inside C++.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
STAGES = ("initialize_ms", "cost_map_ms", "search_ms", "simplify_ms", "optimize_ms", "validate_ms", "total_ms")


def write_json(path, data):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def nonnegative(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return number


def seed_value(value):
    number = int(value)
    if not 0 <= number <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("must be between 0 and 4294967295")
    return number


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_rows(directory):
    path = directory / "results.csv"
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            # A terminated process can leave its last CSV row incomplete.
            try:
                valid = (None not in row and all(value is not None for value in row.values())
                         and row["passed"] in ("0", "1") and int(row["case_id"]) > 0
                         and math.isfinite(float(row["total_ms"])))
            except (KeyError, TypeError, ValueError):
                valid = False
            if valid:
                rows.append(row)
        return rows


def selected_cases(inputs, args, rows):
    cases = inputs["cases"]
    if args.failed_only:
        failed = {int(row["case_id"]) for row in rows if row["passed"] == "0"}
        cases = [case for case in cases if case["id"] in failed]
    if args.case_id:
        cases = [case for case in cases if case["id"] == args.case_id]
    if args.from_case is not None:
        cases = [case for case in cases if case["id"] >= args.from_case]
    if not cases:
        raise ValueError("No matching cases; check --case-id, --from-case, and --failed-only")
    return cases


def summarize(directory, manifest):
    rows = read_rows(directory)
    inputs_path = directory / "inputs.json"
    if inputs_path.exists():
        inputs = read_json(inputs_path)
        manifest["seed"] = int(inputs["seed"])
        manifest["parameters"] = {k: v for k, v in inputs.items() if k != "cases"}
        with (directory / "cases.csv").open("w", newline="", encoding="utf-8") as stream:
            fields = ["case_id", "group", "backend", "start_x", "start_y", "end_x", "end_y", "world_start_x", "world_start_y", "world_end_x", "world_end_y"]
            out = csv.DictWriter(stream, fields)
            out.writeheader()
            for case in inputs["cases"]:
                flip = (inputs["height"] - 1) * inputs["resolution"]
                out.writerow(dict(case_id=case["id"], group=case["group"], backend=case["backend"],
                    start_x=case["start_x"], start_y=case["start_y"], end_x=case["end_x"], end_y=case["end_y"],
                    world_start_x=case["start_x"], world_start_y=flip-case["start_y"],
                    world_end_x=case["end_x"], world_end_y=flip-case["end_y"]))
    groups = {"all": rows}
    for row in rows:
        groups.setdefault(row["group"], []).append(row)
        groups.setdefault(row["group"] + "/" + row["backend"], []).append(row)
    groups["ordinary_all"] = [r for r in rows if r["group"] in ("fixed", "random")]
    summary, timing_rows, failure_rows = [], [], []
    for name, subset in groups.items():
        if not subset:
            continue
        passed = sum(row["passed"] == "1" for row in subset)
        ordinary = [r for r in subset if r["group"] in ("fixed", "random")]
        reached = sum(r["frontend_success"] == "1" for r in ordinary)
        attempted = [r for r in ordinary if r["backend_attempted"] == "1"]
        backend_pass = sum(r["passed"] == "1" for r in attempted)
        summary.append(dict(group=name, count=len(subset), passed=passed, failed=len(subset)-passed,
            pass_rate_pct=100*passed/len(subset), frontend_pass_rate_pct=100*reached/len(ordinary) if ordinary else "",
            backend_attempted=len(attempted), backend_conditional_pass_rate_pct=100*backend_pass/len(attempted) if attempted else "",
            backend_solve_success_rate_pct=100*sum(r["backend_success"] == "1" for r in attempted)/len(attempted) if attempted else ""))
        reasons = {}
        for row in subset:
            if row["passed"] == "0":
                key = (row["reason"], row["backend_failure"], row["iteration_limit_reached"], row["time_limit_reached"])
                reasons[key] = reasons.get(key, 0) + 1
        for key, count in sorted(reasons.items()):
            failure_rows.append(dict(group=name, reason=key[0], backend_failure=key[1],
                iteration_limit_reached=key[2], time_limit_reached=key[3], count=count))
        for outcome in ("all", "passed", "failed"):
            selected = subset if outcome == "all" else [r for r in subset if (r["passed"] == "1") == (outcome == "passed")]
            for stage in STAGES:
                values = sorted(float(r[stage]) for r in selected if r.get(stage, "") != "")
                if not values:
                    continue
                timing_rows.append(dict(group=name, outcome=outcome, stage=stage, count=len(values),
                    sum_ms=sum(values), mean_ms=sum(values)/len(values),
                    p95_ms=values[math.ceil(.95*len(values))-1], max_ms=values[-1]))
    def table(filename, records, fields):
        with (directory / filename).open("w", newline="", encoding="utf-8") as stream:
            out = csv.DictWriter(stream, fields)
            out.writeheader()
            out.writerows(records)
    table("summary.csv", summary, ["group", "count", "passed", "failed", "pass_rate_pct", "frontend_pass_rate_pct", "backend_attempted", "backend_conditional_pass_rate_pct", "backend_solve_success_rate_pct"])
    table("timings.csv", timing_rows, ["group", "outcome", "stage", "count", "sum_ms", "mean_ms", "p95_ms", "max_ms"])
    table("failures.csv", failure_rows, ["group", "reason", "backend_failure", "iteration_limit_reached", "time_limit_reached", "count"])
    with (directory / "paths.csv").open("w", newline="", encoding="utf-8") as paths, (directory / "corridors.csv").open("w", newline="", encoding="utf-8") as corridors:
        pw, cw = csv.writer(paths), csv.writer(corridors)
        pw.writerow(["case_id", "kind", "index", "x", "y"])
        cw.writerow(["case_id", "index", "xmin", "ymin", "xmax", "ymax"])
        for row in rows:
            record_path = directory / "cases" / (row["case_id"] + ".json")
            if not record_path.exists():
                continue
            record = read_json(record_path)
            for kind in ("front", "path"):
                for i, point in enumerate(record[kind]):
                    pw.writerow([row["case_id"], kind, i, point["x"], point["y"]])
            for i, box in enumerate(record["corridors"]):
                cw.writerow([row["case_id"], i] + box)
    report = ["# 测试报告", "", f"运行状态：{manifest['status']}；已完成 {len(rows)} 例；种子：{manifest.get('seed', 'unknown')}。",
        f"进程执行及实时日志耗时：{manifest.get('execution_wall_seconds', 0):.3f} s；报告生成耗时见 manifest.json。", "",
        "功能与单次总耗时单位均为 ms，排除读图、绘图、等待和写盘；整体通过率包含无路样例。",
        "backend_conditional_pass_rate 是进入后端后的最终通过率；backend_solve_success_rate 是后端返回 success 的比例，均包含碰撞规则。",
        "OSQP 1.x 状态 7/8 分别表示内部迭代上限/内部超时；其每轴 0.1 秒 setter 在当前依赖中有未修复兼容问题。",
        "随机点取自由栅格中心；所有保存的路径采用内部 X右/Y下米制坐标。", "",
        "| 测试组 | 数量 | 通过 | 失败 | 通过率 |", "| --- | ---: | ---: | ---: | ---: |"]
    for row in summary:
        report.append(f"| {row['group']} | {row['count']} | {row['passed']} | {row['failed']} | {row['pass_rate_pct']:.2f}% |")
    report += ["", "各阶段耗时（包含成功与失败样例；未执行阶段不计入次数）：", "",
        "| 测试组 | 阶段 | 次数 | 累计 ms | 平均 ms | P95 ms | 最大 ms |", "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for row in timing_rows:
        if row["outcome"] == "all" and "/" not in row["group"]:
            report.append(f"| {row['group']} | {row['stage']} | {row['count']} | {row['sum_ms']:.3f} | {row['mean_ms']:.3f} | {row['p95_ms']:.3f} | {row['max_ms']:.3f} |")
    report += ["", "失败原因（两个外层限制可同时达到）：", "", "| 测试组 | 原因 | 后端详情 | 轮数上限 | 时间上限 | 数量 |", "| --- | --- | --- | --- | --- | ---: |"]
    for row in failure_rows:
        if "/" not in row["group"]:
            report.append(f"| {row['group']} | {row['reason']} | {row['backend_failure']} | {row['iteration_limit_reached']} | {row['time_limit_reached']} | {row['count']} |")
    report += ["", "详细成功/失败耗时见 timings.csv；原始轨迹与记录见 cases/。", ""]
    (directory / "summary.md").write_text("\n".join(report), encoding="utf-8")
    manifest["completed_cases"] = len(rows)
    return rows


def replay(directory, inputs, cases):
    # Optional GUI dependency; automatic runs and reporting use the standard library only.
    import cv2
    import numpy as np
    image = cv2.imread(str(directory / "map.png"), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Saved map cannot be read")
    resolution = inputs["resolution"]
    def point(x, y):
        return round(x/resolution), round(y/resolution)
    try:
        for case in cases:
            path = directory / "cases" / f"{case['id']}.json"
            if not path.exists():
                raise ValueError(f"Case {case['id']} has no saved result (run may be incomplete)")
            record = read_json(path)
            canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for box in record["corridors"]:
                cv2.rectangle(canvas, point(box[0], box[1]), point(box[2], box[3]), (255, 0, 0), 1)
            for kind, color in (("front", (0, 128, 0)), ("path", (0, 0, 192))):
                points = np.array([point(p["x"], p["y"]) for p in record[kind] if math.isfinite(p["x"]) and math.isfinite(p["y"])], dtype=np.int32)
                if len(points) > 1:
                    cv2.polylines(canvas, [points], False, color, 1, cv2.LINE_AA)
            for item in case.get("items", []):
                norm = math.hypot(item["vx"], item["vy"])
                if item["use_velocity"] and norm:
                    cv2.arrowedLine(canvas, point(item["x"], item["y"]), point(item["x"]+item["vx"]/norm, item["y"]+item["vy"]/norm), (0, 160, 160), 1)
            for i in range(0, len(record.get("motion", [])), 100):
                motion = record["motion"][i]
                position = record["path"][i]
                norm = math.hypot(motion["vx"], motion["vy"])
                if norm > 1e-6:
                    cv2.arrowedLine(canvas, point(position["x"], position["y"]),
                        point(position["x"]+.5*motion["vx"]/norm, position["y"]+.5*motion["vy"]/norm), (0, 160, 160), 1)
            for key, color in (("start", (0, 200, 0)), ("end", (255, 0, 0))):
                cv2.circle(canvas, point(case[key+"_x"], case[key+"_y"]), 3, color, -1)
            scale = min(2, 1200/canvas.shape[1], 800/canvas.shape[0])
            canvas = cv2.resize(canvas, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
            label = f"case {case['id']} {record['reason']} {record['total_ms']:.3f} ms (saved)"
            cv2.putText(canvas, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1)
            print(label, flush=True)
            cv2.imshow("benchmark replay", canvas)
            while True:
                key = cv2.waitKey(50)
                if key >= 0:
                    if key & 255 in (27, ord("q"), ord("Q")):
                        return 0
                    break
                try:
                    if cv2.getWindowImageRect("benchmark replay")[2] <= 0:
                        return 0
                except cv2.error:
                    return 0
    finally:
        cv2.destroyAllWindows()
    return 0


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--executable", type=Path, default=ROOT / "build/play")
    result.add_argument("--map", type=Path)
    result.add_argument("--resolution", type=positive)
    result.add_argument("--count", type=nonnegative)
    result.add_argument("--seed", type=seed_value)
    result.add_argument("--suite", choices=("existing", "random"))
    result.add_argument("--output-dir", type=Path)
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--rerun", type=Path, help="rerun saved inputs; opens manual demo unless --headless")
    mode.add_argument("--replay", type=Path, help="view original saved paths without solving")
    mode.add_argument("--summarize", type=Path, help="regenerate tables for an existing run")
    result.add_argument("--headless", action="store_true", help="rerun without windows")
    result.add_argument("--case-id", type=positive_id)
    result.add_argument("--from-case", type=positive_id)
    result.add_argument("--failed-only", action="store_true")
    result.add_argument("--max-iterations", type=positive_id)
    result.add_argument("--time-limit", type=positive)
    result.add_argument("--max-speed", type=positive)
    result.add_argument("--max-acc", type=positive)
    return result


def positive_id(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def main(argv=None):
    args = parser().parse_args(argv)
    if args.summarize:
        directory = args.summarize.resolve()
        manifest = read_json(directory / "manifest.json")
        summarize(directory, manifest)
        write_json(directory / "manifest.json", manifest)
        print(directory / "summary.md")
        return 0
    source = args.rerun or args.replay
    inputs, selected = None, None
    if source:
        if any(value is not None for value in (args.map, args.resolution, args.count, args.seed, args.suite, args.max_iterations, args.time_limit, args.max_speed, args.max_acc)):
            raise ValueError("Historical modes use saved inputs and parameters; do not pass new map/seed/solver options")
        source = source.resolve()
        manifest_source = read_json(source / "manifest.json")
        if digest(source / "map.png") != manifest_source["map_sha256"]:
            raise ValueError("Saved map checksum mismatch")
        inputs = read_json(source / "inputs.json")
        if manifest_source.get("inputs_sha256") and digest(source / "inputs.json") != manifest_source["inputs_sha256"]:
            raise ValueError("Saved inputs checksum mismatch")
        selected = selected_cases(inputs, args, read_rows(source))
        if args.replay:
            return replay(source, inputs, selected)
    elif args.failed_only:
        raise ValueError("--failed-only requires --rerun or --replay")
    if not source and args.from_case is not None:
        raise ValueError("--from-case is for historical modes")
    executable = args.executable.resolve()
    if not executable.is_file():
        raise ValueError(f"C++ executable missing: {executable}; build with cmake first")
    map_path = source / "map.png" if source else (args.map or ROOT / "images/rmuc2025.png").resolve()
    if not map_path.is_file():
        raise ValueError(f"Map missing: {map_path}")
    directory = (args.output_dir or ROOT / "benchmark-results" / datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f")).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    manifest = dict(schema_version=1, status="running", started_utc=datetime.now(timezone.utc).isoformat(),
        executable=str(executable), executable_sha256=digest(executable), platform=platform.platform(),
        python=sys.version, source_run=str(source) if source else None)
    try:
        manifest["dynamic_libraries"] = subprocess.run(["ldd", str(executable)], capture_output=True, text=True, check=False).stdout
        manifest["compiler"] = subprocess.run(["c++", "--version"], capture_output=True, text=True, check=False).stdout.splitlines()[0]
    except (OSError, IndexError):
        pass
    # Preserve original bytes; OpenCV detects the actual format rather than the extension.
    shutil.copyfile(map_path, directory / "map.png")
    manifest.update(original_map=str(map_path), map_sha256=digest(directory / "map.png"))
    mode = "demo" if args.rerun and not args.headless else "batch"
    command = [str(executable), "--mode", mode, "--map", str(directory / "map.png"), "--output", str(directory)]
    if inputs is not None:
        # Freeze exact generated inputs; seed remains stored and can be used for regeneration separately.
        write_json(directory / "rerun-inputs.json", inputs)
        command += ["--inputs", str(directory / "rerun-inputs.json"), "--case-id", ",".join(str(c["id"]) for c in selected)]
        manifest["requested_case_ids"] = [c["id"] for c in selected]
        if manifest_source.get("executable_sha256") != manifest["executable_sha256"]:
            print("Note: executable differs from historical run; exact inputs are preserved, results may change.", flush=True)
    else:
        command += ["--suite", args.suite or ("random" if args.map else "existing"), "--count", str(args.count if args.count is not None else 1000)]
        for name in ("seed", "resolution", "max_iterations", "time_limit", "max_speed", "max_acc"):
            value = getattr(args, name)
            if value is not None:
                command += ["--" + name.replace("_", "-"), str(value)]
        if args.case_id:
            command += ["--case-id", str(args.case_id)]
            manifest["requested_case_ids"] = [args.case_id]
    manifest["command"] = command
    write_json(directory / "manifest.json", manifest)
    print(f"Results: {directory}", flush=True)
    process, code = None, 2
    execution_start = time.perf_counter()
    try:
        with (directory / "run.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end="", flush=True)
            code = process.wait()
        manifest["returncode"] = code
        manifest["status"] = "complete" if code == 0 else "failed"
    except KeyboardInterrupt:
        manifest["status"] = "interrupted"
        code = 130
    except OSError as error:
        manifest["status"] = "failed"
        manifest["error"] = str(error)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        manifest["execution_wall_seconds"] = time.perf_counter() - execution_start
        completion = directory / "completion.json"
        if completion.exists():
            manifest["completion"] = read_json(completion)
            if manifest["completion"]["stopped"]:
                manifest["status"] = "stopped"
        if (directory / "inputs.json").exists():
            saved = read_json(directory / "inputs.json")
            manifest["inputs_sha256"] = digest(directory / "inputs.json")
            expected = len(manifest.get("requested_case_ids", saved["cases"]))
            manifest["expected_cases"] = expected
            completed_rows = read_rows(directory)
            missing = [r["case_id"] for r in completed_rows if not (directory / "cases" / (r["case_id"]+".json")).is_file()]
            if manifest["status"] == "complete" and (len(completed_rows) != expected or missing or not completion.exists()):
                manifest["status"] = "incomplete"
                code = 2
        elif manifest["status"] == "complete":
            manifest["status"] = "incomplete"
            code = 2
        report_start = time.perf_counter()
        try:
            rows = summarize(directory, manifest)
        except (ValueError, OSError, KeyError) as error:
            rows = read_rows(directory)
            manifest["status"] = "incomplete"
            manifest["report_error"] = str(error)
            code = 2
        finally:
            manifest["report_wall_seconds"] = time.perf_counter() - report_start
            manifest["wall_seconds"] = time.perf_counter() - started
            write_json(directory / "manifest.json", manifest)
    print(f"Report: {directory / 'summary.md'} ({len(rows)} completed, {manifest['status']})", flush=True)
    return code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError, json.JSONDecodeError) as error:
        print(f"benchmark error: {error}", file=sys.stderr)
        sys.exit(2)
