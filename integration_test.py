#!/usr/bin/env python3
"""
无人机编队控制系统 - 跨语言集成测试脚本
测试 Python API 以及 Python-Java 系统交互流程
"""

import json
import time
import sys
import numpy as np

try:
    import requests
except ImportError:
    print("请先安装 requests: pip install requests")
    sys.exit(1)


PYTHON_BASE = "http://localhost:5001"
JAVA_BASE = "http://localhost:8080"

TEST_BLOCKS = [
    {"action": "move_relative", "params": {"dx": 3.0, "dy": 0.0, "dz": 0.0, "duration": 3.0}},
    {"action": "formation_change", "params": {"formation": "circle", "duration": 2.0}},
    {"action": "move_relative", "params": {"dx": 0.0, "dy": 2.0, "dz": 1.0, "duration": 3.0}},
    {"action": "formation_change", "params": {"formation": "v", "duration": 2.0}},
]


def header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def sub(title):
    print(f"\n--- {title} ---")


def check_service_alive(url, name):
    try:
        r = requests.get(f"{url}/api/v1/health", timeout=3)
        print(f"  [OK] {name} 服务在线: {r.status_code}")
        return True
    except Exception as e:
        print(f"  [SKIP] {name} 服务不可用: {e}")
        return False


def test_python_health():
    header("Python 服务健康检查")
    if not check_service_alive(PYTHON_BASE, "Python"):
        return False

    sub("编队列表查询")
    r = requests.get(f"{PYTHON_BASE}/api/v1/formations")
    data = r.json()
    print(f"  支持 {len(data['formations'])} 种编队:")
    for f in data["formations"]:
        print(f"    - {f['name']}: {f['description']}")

    sub("单编队查询 (square_grid)")
    r = requests.get(f"{PYTHON_BASE}/api/v1/formation/square_grid?num_drones=10")
    data = r.json()
    positions = np.array(data["positions"])
    print(f"  形状: {positions.shape}")
    print(f"  碰撞风险: {data['has_collision_risk']}")
    print(f"  第0架位置: {positions[0]}")
    print(f"  相邻无人机最小间距: {_min_pairwise_dist(positions):.3f} m")
    return True


def _min_pairwise_dist(positions):
    min_d = float("inf")
    n = len(positions)
    for i in range(n):
        for j in range(i + 1, n):
            d = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
            if d < min_d:
                min_d = d
    return min_d


def test_python_trajectory_compute():
    header("Python 轨迹矩阵计算 + 碰撞风险拦截")

    sub("根据拖拽代码块 (blocks) 计算轨迹")
    payload = {
        "num_drones": 10,
        "samples_per_segment": 20,
        "safe_distance": 0.5,
        "generate_charts": False,
        "blocks": TEST_BLOCKS,
    }
    r = requests.post(f"{PYTHON_BASE}/api/v1/trajectory/compute", json=payload, timeout=15)
    if r.status_code != 200:
        print(f"  [ERROR] 状态码: {r.status_code}, 响应: {r.text[:200]}")
        return None

    data = r.json()
    traj_id = data["trajectory_id"]
    print(f"  轨迹ID: {traj_id}")
    print(f"  无人机数量: {data['num_drones']}")
    print(f"  总时间步: {data['total_timesteps']}")
    print(f"  总时长: {data['duration_seconds']:.2f} 秒")
    print(f"  采样频率: {data['timestep_hz']:.1f} Hz")

    cr = data["collision_report"]
    print(f"\n  === 碰撞检测报告 ===")
    print(f"  安全距离: {cr['safe_distance']} m")
    print(f"  整体是否有风险: {cr['has_risk']}")
    print(f"  碰撞次数: {cr['collision_count']}")
    print(f"  全局最小间距: {cr['min_distance_overall']:.4f} m")
    if cr["collision_timesteps"]:
        print(f"  发生风险的时间步: {cr['collision_timesteps'][:10]}...")

    for drone_key, preview in list(data["trajectory_preview"].items())[:3]:
        print(f"\n  {drone_key}:")
        print(f"    起点: {preview['start']}")
        print(f"    终点: {preview['end']}")

    sub("按时间步获取单步指令 (模拟 Java 高频取点)")
    for t in [0, 10, 30, 60]:
        r = requests.get(f"{PYTHON_BASE}/api/v1/trajectory/{traj_id}/point/{t}?correction=false")
        if r.status_code != 200:
            continue
        pdata = r.json()
        positions = np.array(pdata["positions"])
        min_d = _min_pairwise_dist(positions)
        print(f"  t={t}  时间={pdata['time_seconds']:.2f}s  无人机{len(positions)}架  最小间距={min_d:.3f}m")

    sub("批量获取轨迹点")
    batch_payload = {"start_step": 0, "end_step": 5, "step": 1, "apply_correction": True}
    r = requests.post(f"{PYTHON_BASE}/api/v1/trajectory/{traj_id}/batch", json=batch_payload)
    bdata = r.json()
    print(f"  返回 {bdata['num_points']} 个时间步的指令")
    print(f"  其中有 {bdata['risk_count']} 个时间步存在碰撞风险")

    return traj_id


def test_python_deviation_and_collision():
    header("Python 偏离距检查 & 碰撞拦截接口")

    sub("偏离距计算接口")
    reference = [[0.0, 0.0, 2.0] for _ in range(10)]
    actual = [[0.0 + i * 0.05, 0.0, 2.0] for i in range(10)]
    actual[5] = [1.0, 0.5, 2.0]
    payload = {
        "reference_positions": reference,
        "actual_positions": actual,
        "warning_threshold": 0.3,
        "emergency_threshold": 0.8,
    }
    r = requests.post(f"{PYTHON_BASE}/api/v1/deviation/check", json=payload)
    ddata = r.json()
    print(f"  状态: {ddata['status']}")
    print(f"  最大偏离: {ddata['max_deviation']:.3f} m")
    print(f"  平均偏离: {ddata['average_deviation']:.3f} m")
    print(f"  RMS偏离:  {ddata['rms_deviation']:.3f} m")
    if ddata["emergencies"]:
        for e in ddata["emergencies"]:
            print(f"    EMERGENCY Drone-{e['drone_id']}: {e['deviation']:.3f}m -> {e['recommended_action']}")
    if ddata["warnings"]:
        for w in ddata["warnings"][:3]:
            print(f"    WARNING   Drone-{w['drone_id']}: {w['deviation']:.3f}m -> {w['recommended_action']}")

    sub("碰撞检测 + 纠偏向量")
    tight_positions = [
        [0.0, 0.0, 2.0], [0.2, 0.0, 2.0], [0.4, 0.0, 2.0],
        [0.0, 1.5, 2.0], [1.0, 1.5, 2.0],
    ] + [[i * 1.0, i * 0.5, 2.0] for i in range(2, 7)]
    payload = {
        "positions": tight_positions,
        "safe_distance": 0.5,
        "return_correction": True,
    }
    r = requests.post(f"{PYTHON_BASE}/api/v1/collision/check", json=payload)
    cdata = r.json()
    print(f"  碰撞风险: {cdata['has_collision_risk']}")
    print(f"  纠偏前风险: {cdata['has_collision_risk']}")
    print(f"  纠偏后风险: {cdata['collision_risk_after_correction']}")
    print(f"  碰撞详情数: {len(cdata['collision_details'])}")
    if cdata["collision_details"]:
        for c in cdata["collision_details"][:3]:
            print(f"    Drone-{c['drone_a']} <-> Drone-{c['drone_b']}: {c['distance']:.3f}m")


def test_java_api(java_online):
    header("Java Spring Boot 后端接口")

    if not java_online:
        print("  Java 服务未启动，跳过 Java 接口测试")
        return

    sub("健康检查")
    r = requests.get(f"{JAVA_BASE}/api/v1/health", timeout=3)
    print(f"  状态: {r.status_code}, 服务: {r.json().get('service')}")

    sub("无人机列表")
    r = requests.get(f"{JAVA_BASE}/api/v1/drones")
    data = r.json()
    if data.get("success"):
        drones = data["data"]
        print(f"  注册无人机数量: {len(drones)}")
        for d in drones[:3]:
            print(f"    Drone-{d['droneIndex']}: {d['droneCode']} [{d['status']}]")

    sub("创建轨迹任务 (调用 Python 计算并存储)")
    payload = {
        "missionName": "少儿夏令营试飞任务",
        "numDrones": 10,
        "samplesPerSegment": 15,
        "safeDistance": 0.5,
        "generateCharts": False,
        "blocks": TEST_BLOCKS,
    }
    r = requests.post(f"{JAVA_BASE}/api/v1/trajectory/missions", json=payload, timeout=30)
    if r.status_code != 200:
        print(f"  [ERROR] 状态码: {r.status_code}")
        print(f"  响应: {r.text[:500]}")
        return

    result = r.json()
    mission = result["data"]
    print(f"  任务ID: {mission['missionId']}")
    print(f"  Python轨迹ID: {mission['pythonTrajectoryId']}")
    print(f"  时间步总数: {mission['totalTimesteps']}")
    print(f"  总时长: {mission['durationSeconds']:.2f}s")
    print(f"  碰撞风险: {mission['hasCollisionRisk']}")
    print(f"  任务状态: {mission['status']}")
    if not result.get("success"):
        print(f"  ! 提示: {result.get('message')}")

    mission_id = mission["missionId"]

    sub("仿真轨迹下发: 单步取点 (模拟高频下发 20Hz)")
    for t in [0, 5, 10, 20]:
        r = requests.get(f"{JAVA_BASE}/api/v1/trajectory/missions/{mission_id}/commands/{t}")
        if r.status_code != 200:
            print(f"  t={t}: [FAIL]")
            continue
        cb = r.json()["data"]
        cmds = cb["commands"]
        positions = [[c["targetX"], c["targetY"], c["targetZ"]] for c in cmds]
        min_d = _min_pairwise_dist(positions)
        print(f"  t={t}  time={cb['timeSeconds']:.2f}s  "
              f"risk={cb['hasCollisionRisk']}  drones={len(cmds)}  "
              f"minDist={min_d:.3f}m")

    sub("顺序取下一个指令批次 (游标模式)")
    r = requests.post(f"{JAVA_BASE}/api/v1/trajectory/missions/{mission_id}/reset")
    for _ in range(3):
        r = requests.get(f"{JAVA_BASE}/api/v1/trajectory/missions/{mission_id}/next?applyCorrection=true")
        if r.status_code == 200:
            cb = r.json()["data"]
            print(f"  游标当前 t={cb['timestep']}, 风险={cb['hasCollisionRisk']}")

    sub("实际 GPS/UWB 坐标上报 + 偏离距记录")
    import random
    positions_list = []
    r0 = requests.get(f"{JAVA_BASE}/api/v1/trajectory/missions/{mission_id}/commands/10")
    if r0.status_code == 200:
        cmds0 = r0.json()["data"]["commands"]
        for c in cmds0:
            noise_x = random.gauss(0, 0.08)
            noise_y = random.gauss(0, 0.08)
            noise_z = random.gauss(0, 0.04)
            positions_list.append({
                "droneIndex": c["droneIndex"],
                "droneCode": c["droneCode"],
                "posX": c["targetX"] + noise_x,
                "posY": c["targetY"] + noise_y,
                "posZ": c["targetZ"] + noise_z,
                "signalSource": "UWB",
                "timestamp": int(time.time() * 1000),
            })
        positions_list[3]["posX"] += 0.5
        positions_list[3]["posY"] += 0.3

    report = {
        "missionId": mission_id,
        "timestep": 10,
        "positions": positions_list,
    }
    r = requests.post(f"{JAVA_BASE}/api/v1/deviation/report", json=report)
    if r.status_code == 200 and r.json().get("success"):
        records = r.json()["data"]
        max_dev = max(x["deviationDistance"] for x in records)
        warnings = [x for x in records if x["status"] in ("WARNING", "EMERGENCY")]
        print(f"  已记录 {len(records)} 条偏离数据")
        print(f"  最大偏离: {max_dev:.4f} m")
        print(f"  WARNING/EMERGENCY 数: {len(warnings)}")
        for w in warnings:
            print(f"    Drone-{w['droneIndex']}: {w['deviationDistance']:.3f}m [{w['status']}]")

    sub("偏离距统计摘要")
    r = requests.get(f"{JAVA_BASE}/api/v1/deviation/missions/{mission_id}/summary")
    if r.status_code == 200 and r.json().get("success"):
        summary = r.json()["data"]
        print(f"  总记录数: {summary['totalRecords']}")
        print(f"  最大偏离: {summary['maxDeviation']:.4f} m")
        print(f"  平均偏离: {summary['avgDeviation']:.4f} m")
        print(f"  WARNING 数: {summary['warningCount']}")
        print(f"  EMERGENCY 数: {summary['emergencyCount']}")
        print(f"  整体状态: {summary['overallStatus']}")
        if summary.get("perDroneStats"):
            for s in summary["perDroneStats"][:5]:
                print(f"    Drone-{s['droneIndex']}: max={s['maxDeviation']:.3f}  avg={s['avgDeviation']:.3f}")


def main():
    print("=" * 60)
    print("  少儿编程夏令营无人机编队系统 - 集成测试")
    print("=" * 60)

    python_online = check_service_alive(PYTHON_BASE, "Python (Flask)")
    java_online = check_service_alive(JAVA_BASE, "Java (Spring Boot)")

    if python_online:
        test_python_health()
        traj_id = test_python_trajectory_compute()
        test_python_deviation_and_collision()
    else:
        print("\n[提示] 启动 Python 服务:")
        print("  cd drone-sim-python && pip install -r requirements.txt && python app.py")

    test_java_api(java_online)
    if not java_online:
        print("\n[提示] 启动 Java 服务 (需先启动 Python):")
        print("  cd drone-control-java && mvn spring-boot:run -Dspring-boot.run.profiles=dev")

    print("\n" + "=" * 60)
    print("  测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
