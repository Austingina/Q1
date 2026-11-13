"""10 秒单帧单推理线路 B 交接的纯 MuJoCo 鲁棒性回归，不会连接真机。"""

from __future__ import annotations

import json

from apps.line_b_web.handoff_sim_check import run_case


def main() -> None:
    scenarios = [
        ("hold_0.5s", 0.5, 0.0, 0),
        ("hold_2s", 2.0, 0.0, 0),
        ("hold_5s", 5.0, 0.0, 0),
        ("hold_10s", 10.0, 0.0, 0),
        ("joint_noise_0.01_seed1", 5.0, 0.01, 1),
        ("joint_noise_0.01_seed2", 5.0, 0.01, 2),
        ("joint_noise_0.02_seed3", 5.0, 0.02, 3),
        ("joint_noise_0.02_seed4", 5.0, 0.02, 4),
    ]
    results = []
    for name, hold_s, perturb, seed in scenarios:
        result = run_case(10.0, hold_s=hold_s, joint_perturb_rad=perturb, perturb_seed=seed)
        result["scenario"] = name
        results.append(result)
    passed = all(r["passed"] for r in results)
    print(json.dumps({"offline_only": True, "passed": passed, "results": results}, ensure_ascii=False, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
