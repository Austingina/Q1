"""Measure ONNX inference latency without creating a robot backend.

This probe deliberately uses synthetic all-zero inputs.  It never imports ROS,
opens a control topic, or calls ``Robot.reset``/``Robot.step``.  Run each
scenario in a fresh process so inactive ONNX sessions from an earlier scenario
cannot affect the result.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np


GRAPH_PATHS = {
    "tele": Path("weights/dual_v3b/policy_tele.onnx"),
    "ref": Path("weights/dual_v3b/policy_ref.onnx"),
    "decoder": Path("weights/dual_v3b/decoder.onnx"),
    "skill": Path("weights/getup_vpd_ft/policy.onnx"),
    "dance": Path("weights/q1_smooth_dance/policy.onnx"),
}

SCENARIOS = {
    "skill-only": (["skill"], "skill"),
    "tele-only": (["tele"], "tele"),
    "base-loaded": (["tele", "ref", "decoder"], "tele"),
    "full-loaded-skill": (["tele", "ref", "decoder", "skill", "dance"], "skill"),
    "full-loaded-tele": (["tele", "ref", "decoder", "skill", "dance"], "tele"),
}


def _session(path: Path, ort_mode: str):
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    if ort_mode == "sequential-no-spin":
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        options.add_session_config_entry("session.inter_op.allow_spinning", "0")
    return ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])


def _input_array(node) -> np.ndarray:
    shape = [1 if not isinstance(dim, int) or dim <= 0 else dim for dim in node.shape]
    dtypes = {
        "tensor(float)": np.float32,
        "tensor(double)": np.float64,
        "tensor(int64)": np.int64,
        "tensor(int32)": np.int32,
        "tensor(bool)": np.bool_,
    }
    try:
        dtype = dtypes[node.type]
    except KeyError as exc:
        raise ValueError(f"unsupported ONNX input type: {node.type}") from exc
    return np.zeros(shape, dtype=dtype)


def run_probe(repo_root: Path, scenario: str, ort_mode: str, warmup: int, samples: int) -> dict:
    graph_names, target_name = SCENARIOS[scenario]
    sessions = {
        name: _session(repo_root / GRAPH_PATHS[name], ort_mode)
        for name in graph_names
    }
    target = sessions[target_name]
    inputs = {node.name: _input_array(node) for node in target.get_inputs()}

    for _ in range(warmup):
        target.run(None, inputs)

    elapsed_ms = []
    for _ in range(samples):
        started = time.perf_counter_ns()
        target.run(None, inputs)
        elapsed_ms.append((time.perf_counter_ns() - started) / 1_000_000.0)

    timings = np.asarray(elapsed_ms, dtype=np.float64)
    return {
        "scenario": scenario,
        "ort_mode": ort_mode,
        "loaded_graphs": graph_names,
        "target_graph": target_name,
        "warmup": warmup,
        "samples": samples,
        "mean_ms": float(timings.mean()),
        "p50_ms": float(np.percentile(timings, 50)),
        "p90_ms": float(np.percentile(timings, 90)),
        "p99_ms": float(np.percentile(timings, 99)),
        "max_ms": float(timings.max()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="repository root containing weights/",
    )
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--ort-mode", choices=("current", "sequential-no-spin"), default="current")
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--samples", type=int, default=100)
    args = parser.parse_args()

    if args.warmup < 0 or args.samples <= 0:
        parser.error("--warmup must be >= 0 and --samples must be > 0")
    result = run_probe(args.repo_root.resolve(), args.scenario, args.ort_mode, args.warmup, args.samples)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
