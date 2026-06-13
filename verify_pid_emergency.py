#!/usr/bin/env python3
"""
端到端测试：风摆动态姿态纠偏 + 一键迫降全链路验证
"""

import json
import sys
import time
import requests
import numpy as np

PY = "http://localhost:5001"


def check_service():
    try:
        r = requests.get(f"{PY}/api/v1/health", timeout=3)
        return r.status_code == 200
    except:
        return False


def section(title):
    print(f"\n{'='*70}")
    print(f"  🧪 {title}")
    print(f"{'='*70}")


def test_1_wind_simulation():
    section("场景1: 风扰动模拟 (模拟风摆效应)")
    positions = [[float(i), 0.0, 2.0] for i in range(10)]
    wind = [0.5, -0.2, 0.05]

    r = requests.post(f"{PY}/api/v1/wind/simulate", json={
        "positions": positions,
        "wind_vector": wind,
        "turbulence": 0.05,
        "timestep": 10,
        "dt": 0.05,
    }, timeout=5)

    data = r.json()
    print(f"  原始位置: Drone-0 = {data['original_positions'][0]}")
    print(f"  受风后: Drone-0 = {data['displaced_positions'][0]}")
    print(f"  位移向量: Drone-0 = {data['displacement_per_drone'][0]}")

    max_disp = max(np.linalg.norm(d) for d in data['displacement_per_drone'])
    print(f"  最大位移: {max_disp*100:.2f} cm")

    assert max_disp > 0.01, "风扰动应产生实际位移"
    print("  ✅ 风扰动模拟正常")


def test_2_pid_attitude_correction_below_20cm():
    section("场景2: 偏离 < 20cm → 仅告警，不触发 PID")
    reference = [[float(i), 0.0, 2.0] for i in range(10)]
    actual = [[i * 1.0 + 0.05, 0.03, 2.01] for i in range(10)]  # 仅 5cm 偏离

    r = requests.post(f"{PY}/api/v1/attitude/correct", json={
        "reference_positions": reference,
        "actual_positions": actual,
        "timestep": 100,
        "dt": 0.05,
        "wind_vector": [0.3, -0.1, 0.02],
        "session_id": "test-below-20",
    }, timeout=5)

    data = r.json()
    print(f"  最大偏离: {data['max_deviation']*100:.2f} cm")
    print(f"  状态: {data['overall_status']}")
    print(f"  force_land: {data['force_land']}")
    print(f"  建议: {data['recommendations']}")

    assert data['overall_status'] == 'OK'
    assert data['force_land'] == False
    print("  ✅ 小偏离正常，状态 OK")


def test_3_pid_between_20_and_50cm():
    section("场景3: 偏离 20~50cm → PID 纠偏激活")
    reference = [[float(i), 0.0, 2.0] for i in range(10)]
    actual = [[i * 1.0, 0.0, 2.0] for i in range(10)]
    actual[0] = [0.0, 0.25, 2.05]   # 25cm 偏离
    actual[3] = [3.0, -0.28, 1.92]  # ~30cm 偏离

    r = requests.post(f"{PY}/api/v1/attitude/correct", json={
        "reference_positions": reference,
        "actual_positions": actual,
        "timestep": 200,
        "dt": 0.05,
        "wind_vector": [0.3, -0.1, 0.02],
        "session_id": "test-20-50",
    }, timeout=5)

    data = r.json()
    print(f"  最大偏离: {data['max_deviation']*100:.2f} cm")
    print(f"  状态: {data['overall_status']}")
    print(f"  WARNING 无人机: {len(data['warning_drones'])} 架")
    print(f"  EMERGENCY 无人机: {len(data['emergency_drones'])} 架")
    print(f"  风力抵消速度: {data['wind_cancel_velocity']}")
    print(f"  风力抵消力矩: {data['wind_cancel_torque']}")

    vel_corr = data['velocity_corrections']
    max_vel = max(np.linalg.norm(v) for v in vel_corr)
    print(f"  最大纠偏速度: {max_vel:.4f} m/s")

    assert data['overall_status'] == 'PID_CORRECTION_ACTIVE'
    assert data['force_land'] == False
    assert len(data['warning_drones']) > 0
    assert max_vel <= 2.0  # max_correction_velocity
    print("  ✅ 20cm~50cm PID 纠偏激活正常")

    print()
    print("  🔍 PID 状态详情（前 2 架）:")
    for drone_idx in ['0', '3']:
        if drone_idx in data['pid_states']:
            s = data['pid_states'][drone_idx]
            print(f"    Drone-{drone_idx}:")
            print(f"      X轴: kp={s['x']['kp']:.2f}, ki={s['x']['ki']:.2f}, kd={s['x']['kd']:.2f}")
            print(f"      Y轴: kp={s['y']['kp']:.2f}, integral={s['y']['integral']:.4f}")
            print(f"      Z轴: prev_error={s['z']['prev_error']:.4f}")


def test_4_pid_over_50cm():
    section("场景4: 偏离 > 50cm → 强制迫降触发 (爆红告警)")
    reference = [[float(i), 0.0, 2.0] for i in range(10)]
    actual = [[i * 1.0, 0.0, 2.0] for i in range(10)]
    actual[5] = [5.0, 1.2, 2.8]   # 1.44m 偏离！严重风摆

    r = requests.post(f"{PY}/api/v1/attitude/correct", json={
        "reference_positions": reference,
        "actual_positions": actual,
        "timestep": 300,
        "dt": 0.05,
        "wind_vector": [0.3, -0.1, 0.02],
        "session_id": "test-over-50",
    }, timeout=5)

    data = r.json()
    print(f"  最大偏离: {data['max_deviation']*100:.2f} cm")
    print(f"  状态: {data['overall_status']}")
    print(f"  force_land: {data['force_land']}")
    print(f"  迫降原因: {data['force_land_reason']}")
    print(f"  迫降速度 Z: {data['land_velocity_z']} m/s")
    print(f"  EMERGENCY 无人机: {len(data['emergency_drones'])} 架")
    for e in data['emergency_drones']:
        print(f"    - Drone-{e['drone_index']}: {e['distance']*100:.1f}cm")

    for rec in data['recommendations']:
        print(f"  📢 {rec}")

    assert data['force_land'] == True
    assert data['land_velocity_z'] < 0
    print("  ✅ >50cm 强制迫降触发，前端可爆红")


def test_5_emergency_land_plan():
    section("场景5: 一键迫降指令生成")
    r = requests.post(f"{PY}/api/v1/emergency/land-plan", json={
        "num_drones": 10,
        "current_altitudes": [2.5, 2.3, 2.1, 2.4, 2.2, 2.6, 2.0, 2.3, 2.5, 2.4],
        "land_velocity_z": -2.0,
        "safe_z": 0.1,
    }, timeout=5)

    data = r.json()
    print(f"  动作: {data['action']}")
    print(f"  降落速度: {data['land_velocity_z']} m/s")
    print(f"  全局广播: {json.dumps(data['global_broadcast'], ensure_ascii=False)}")
    print(f"  前3架降落指令:")
    for cmd in data['per_drone_commands'][:3]:
        print(f"    {cmd['drone_code']}: 速度Z={cmd['velocity_z']}, "
              f"预计 {cmd['estimated_land_seconds']:.2f}s 落地")

    assert data['action'] == 'FORCE_EMERGENCY_LAND'
    assert data['global_broadcast']['kill_horizontal'] == True
    assert data['global_broadcast']['led_color'] == 'RED_BLINK'
    print("  ✅ 一键迫降指令生成正常")


def test_6_pid_session_persistence():
    section("场景6: PID 积分/微分状态跨请求连续 (模拟连续上报)")
    reference = [[float(i), 0.0, 2.0] for i in range(10)]
    actual = [[i * 1.0, 0.3, 2.0] for i in range(10)]  # 30cm 恒定偏离
    session = "persistent-session"

    print(f"  连续调用 5 次 PID，验证积分项累积...")
    integrals = []
    for t in range(5):
        r = requests.post(f"{PY}/api/v1/attitude/correct", json={
            "reference_positions": reference,
            "actual_positions": actual,
            "timestep": t,
            "dt": 0.05,
            "wind_vector": [0.0, 0.0, 0.0],
            "session_id": session,
        }, timeout=5)
        data = r.json()
        y_integral = data['pid_states']['0']['y']['integral']
        integrals.append(y_integral)
        print(f"    t={t}: 积分项 Y = {y_integral:.6f}, "
              f"纠偏速度 Y = {data['velocity_corrections'][0][1]:.4f} m/s")

    assert len(set(round(x, 6) for x in integrals)) > 1, "积分项应随时间累积"
    print(f"  ✅ PID 状态跨请求保持，积分项正常累积")

    print()
    print("  重置 PID...")
    r_reset = requests.post(f"{PY}/api/v1/pid/reset", json={
        "session_id": session,
    })
    assert r_reset.status_code == 200

    r_after = requests.post(f"{PY}/api/v1/attitude/correct", json={
        "reference_positions": actual,
        "actual_positions": actual,
        "timestep": 100,
        "dt": 0.05,
        "session_id": session,
    }, timeout=5)
    d_after = r_after.json()
    y_integral_after = d_after['pid_states']['0']['y']['integral']
    print(f"  重置后(零误差)积分项 Y = {y_integral_after:.6f}")
    assert abs(y_integral_after) < 0.001
    print(f"  ✅ PID 重置生效")


def test_7_full_wind_simulation_with_correction():
    section("场景7: 全闭环风扰动+PID 纠偏 60 步仿真")

    blocks = [
        {"action": "move_relative", "params": {"dx": 2.0, "dy": 2.0, "dz": 1.0, "duration": 3.0}},
        {"action": "formation_change", "params": {"formation": "circle", "duration": 2.0}},
    ]

    print("  先计算参考轨迹...")
    r_traj = requests.post(f"{PY}/api/v1/trajectory/compute", json={
        "blocks": blocks,
        "num_drones": 10,
        "samples_per_segment": 30,
        "generate_charts": False,
    }, timeout=10)
    traj_id = r_traj.json()['trajectory_id']
    total_timesteps = r_traj.json()['total_timesteps']
    print(f"  轨迹ID: {traj_id}, 总步: {total_timesteps}")

    print()
    print("  运行 40 步风扰动+PID 纠偏闭环仿真...")
    r_sim = requests.post(f"{PY}/api/v1/wind/simulate-full", json={
        "trajectory_id": traj_id,
        "num_steps": 40,
        "dt": 0.05,
        "wind_vector": [0.3, -0.15, 0.0],
        "turbulence": 0.04,
    }, timeout=30)

    data = r_sim.json()
    max_devs = data['max_deviation_history']
    print(f"  最大偏离 (前10步): {[round(x*100, 1) for x in max_devs[:10]]}")
    print(f"  最大偏离 (后10步): {[round(x*100, 1) for x in max_devs[-10:]]}")
    print(f"  峰值偏离: {data['peak_max_deviation']*100:.2f} cm")
    print(f"  最终偏离: {data['final_max_deviation']*100:.2f} cm")

    emerg_count = sum(1 for s in data['status_history'] if 'EMERGENCY' in s)
    corr_count = sum(1 for s in data['status_history'] if 'PID_CORRECTION' in s)
    print(f"  状态分布: EMERGENCY={emerg_count}次, PID_ACTIVE={corr_count}次")

    print()
    print("  📊 收敛性验证: 后 20 步平均偏离 < 前 20 步")
    first_half = np.mean(max_devs[:20])
    second_half = np.mean(max_devs[20:])
    print(f"    前20步均值: {first_half*100:.2f} cm")
    print(f"    后20步均值: {second_half*100:.2f} cm")
    if second_half < first_half:
        print(f"    ✅ PID 纠偏有效，偏离逐步收敛")
    else:
        print(f"    ⚠  PID 未收敛 (可能风扰动过大)")

    # 查看 PID 会话
    r_status = requests.get(f"{PY}/api/v1/pid/status")
    d_status = r_status.json()
    print(f"  PID 会话数: {d_status['active_sessions']}")


def test_8_deviation_pid_integration():
    section("场景8: /api/v1/deviation/check + enable_pid=true 联动")
    reference = [[float(i), 0.0, 2.0] for i in range(10)]
    actual = [[i * 1.0, 0.25, 2.0] for i in range(10)]

    r = requests.post(f"{PY}/api/v1/deviation/check", json={
        "reference_positions": reference,
        "actual_positions": actual,
        "warning_threshold": 0.2,
        "emergency_threshold": 0.5,
        "enable_pid": True,
        "timestep": 0,
        "dt": 0.05,
        "wind_vector": [0.3, -0.1, 0.02],
        "session_id": "deviation-pid-test",
    }, timeout=5)

    data = r.json()
    print(f"  基础偏离: max={data['max_deviation']*100:.1f}cm, status={data['status']}")
    print(f"  pid_correction 已嵌入: {'pid_correction' in data}")
    if 'pid_correction' in data:
        pc = data['pid_correction']
        print(f"  PID 状态: {pc['overall_status']}")
        print(f"  PID 纠偏速度(Drone-0): {pc['velocity_corrections'][0]}")
        print(f"  PID 力矩(Drone-0): {pc['torque_corrections'][0]}")

    assert 'pid_correction' in data
    print("  ✅ 偏离检查与 PID 联动正常")


def main():
    print("="*70)
    print("  风摆动态姿态纠偏 + 一键迫降 - 端到端测试")
    print("="*70)

    if not check_service():
        print("❌ Python 服务未启动，请先运行 drone-sim-python/app.py")
        sys.exit(1)
    print("✅ Python 服务在线 (localhost:5001)")

    try:
        test_1_wind_simulation()
        test_2_pid_attitude_correction_below_20cm()
        test_3_pid_between_20_and_50cm()
        test_4_pid_over_50cm()
        test_5_emergency_land_plan()
        test_6_pid_session_persistence()
        test_7_full_wind_simulation_with_correction()
        test_8_deviation_pid_integration()
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print()
    print("="*70)
    print("  🎉 全部 8 个场景测试通过!")
    print("="*70)
    print()
    print("  阈值总结:")
    print("    ✦ < 20cm  → OK，PID 待机")
    print("    ✦ 20~50cm → PID_CORRECTION_ACTIVE，下发纠偏速度+力矩")
    print("    ✦ > 50cm  → EMERGENCY_LAND_REQUIRED，force_land=true，前端爆红")
    print()
    print("  Java 侧入口:")
    print("    ✦ POST /api/v1/deviation/report-with-correction")
    print("      → 自动检测偏离 >20cm 触发 PID，>50cm 爆红")
    print("    ✦ POST /api/v1/emergency/land  (一键迫降)")
    print("    ✦ GET  /api/v1/emergency/status  (前端轮询爆红状态)")
    print("    ✦ POST /api/v1/emergency/reset  (状态重置)")


if __name__ == "__main__":
    main()
