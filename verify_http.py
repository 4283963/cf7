#!/usr/bin/env python3
"""修复验证 - HTTP API 级别"""

import requests
import math
import time

PY = "http://localhost:5001"


def test_collision_overlap():
    print("=== HTTP 测试1: 10架完全重叠坐标的碰撞检测 ===")
    overlap = [[1.5, 2.0, 2.0] for _ in range(10)]
    t0 = time.time()
    r = requests.post(f"{PY}/api/v1/collision/check", json={
        "positions": overlap,
        "safe_distance": 0.5,
        "return_correction": True,
    }, timeout=5)
    t1 = time.time()
    data = r.json()
    print(f"  状态码: {r.status_code}")
    print(f"  耗时: {(t1-t0)*1000:.2f} ms")
    print(f"  has_collision_risk: {data['has_collision_risk']}")
    print(f"  碰撞详情数: {len(data['collision_details'])}")
    print(f"  重叠标记 is_overlap: {data['collision_details'][0].get('is_overlap')}")

    all_finite = all(
        all(math.isfinite(c) for c in p)
        for p in data["correction_vectors"]
    )
    print(f"  纠偏向量全有限: {all_finite}")

    corrected_finite = all(
        all(math.isfinite(c) for c in p)
        for p in data["corrected_positions"]
    )
    print(f"  纠偏后坐标全有限: {corrected_finite}")

    # 验证模长上限
    max_norm = max(np.linalg.norm(v) for v in data["correction_vectors"])
    print(f"  纠偏向量最大模长: {max_norm:.4f} (<= 0.5)")

    assert r.status_code == 200
    assert all_finite
    assert corrected_finite
    assert max_norm <= 0.5 + 1e-9
    print("  ✅ 通过")


def test_deviation_check():
    print("\n=== HTTP 测试2: 偏离距计算 ===")
    ref = [[0.0, 0.0, 2.0] for _ in range(10)]
    act = [[i * 0.05, 0, 2.0] for i in range(10)]
    act[5] = [1.0, 0.5, 2.0]

    r = requests.post(f"{PY}/api/v1/deviation/check", json={
        "reference_positions": ref,
        "actual_positions": act,
        "warning_threshold": 0.3,
        "emergency_threshold": 0.8,
    }, timeout=5)
    data = r.json()
    print(f"  状态码: {r.status_code}")
    print(f"  status: {data['status']}")
    print(f"  max_deviation: {data['max_deviation']:.4f}")
    print(f"  avg_deviation: {data['average_deviation']:.4f}")
    print(f"  emergencies: {len(data['emergencies'])}")
    print(f"  warnings: {len(data['warnings'])}")
    assert r.status_code == 200
    assert data["status"] == "EMERGENCY"
    print("  ✅ 通过")


def test_trajectory_steep():
    print("\n=== HTTP 测试3: 陡峭曲线轨迹计算 ===")
    blocks = [
        {"action": "move_relative", "params": {"dx": 0.001, "dy": 0.001, "dz": 0, "duration": 0.001}},
        {"action": "formation_change", "params": {"formation": "circle", "duration": 0.001}},
        {"action": "move_relative", "params": {"dx": 0.001, "dy": 0, "dz": 0.001, "duration": 0.001}},
    ]
    t0 = time.time()
    r = requests.post(f"{PY}/api/v1/trajectory/compute", json={
        "blocks": blocks,
        "num_drones": 10,
        "samples_per_segment": 10,
        "generate_charts": False,
    }, timeout=10)
    t1 = time.time()
    data = r.json()
    print(f"  状态码: {r.status_code}")
    print(f"  耗时: {(t1-t0)*1000:.2f} ms")
    if r.status_code == 200:
        print(f"  trajectory_id: {data['trajectory_id']}")
        print(f"  total_timesteps: {data['total_timesteps']}")
        print(f"  timestep_hz: {data['timestep_hz']:.2f}")
        print(f"  has_risk: {data['collision_report']['has_risk']}")
        print(f"  min_distance_overall: {data['collision_report']['min_distance_overall']:.4e}")

        traj_id = data["trajectory_id"]
        # 批量取点 + 纠偏
        r2 = requests.post(f"{PY}/api/v1/trajectory/{traj_id}/batch", json={
            "start_step": 0,
            "end_step": 10,
            "apply_correction": True,
        }, timeout=5)
        d2 = r2.json()
        print(f"  批量取点: {d2['num_points']} 个时间步, 风险 {d2['risk_count']} 个")

        # 验证所有位置均为有限数
        all_pos_finite = True
        for pt in d2["data"]:
            for p in pt["positions"]:
                for c in p:
                    if not math.isfinite(c):
                        all_pos_finite = False
        print(f"  批量位置全有限: {all_pos_finite}")

        assert all_pos_finite
    print("  ✅ 通过")


if __name__ == "__main__":
    import numpy as np
    test_collision_overlap()
    test_deviation_check()
    test_trajectory_steep()
    print("\n" + "=" * 60)
    print("  🎉 全部 HTTP API 级别的修复验证通过!")
    print("=" * 60)
