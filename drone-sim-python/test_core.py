import numpy as np
from formations import get_formation
from trajectory import build_trajectory_from_blocks
from collision import check_trajectory_collisions


def test_formations():
    print("=== 测试编队生成 ===")
    form = get_formation("square_grid", 10)
    print(f"  方形编队形状: {form.shape}")
    print(f"  第0架初始位置: {form[0]}")
    
    form_c = get_formation("circle", 10)
    print(f"  圆形编队形状: {form_c.shape}")
    
    form_v = get_formation("v", 10)
    print(f"  V形编队形状: {form_v.shape}")
    print("  ✓ 编队生成测试通过")


def test_trajectory():
    print("\n=== 测试轨迹生成 ===")
    blocks = [
        {"action": "move_relative", "params": {"dx": 2.0, "dy": 0, "dz": 0, "duration": 3.0}},
        {"action": "formation_change", "params": {"formation": "circle", "duration": 2.0}},
        {"action": "move_relative", "params": {"dx": 0, "dy": 3.0, "dz": 1.0, "duration": 3.0}},
    ]
    traj, times = build_trajectory_from_blocks(blocks, num_drones=10, samples_per_segment=20)
    print(f"  轨迹张量形状: {traj.shape}")
    print(f"  时间步数: {len(times)}")
    print(f"  总时长: {times[-1]:.2f} 秒")
    
    assert traj.shape[0] == 10, "无人机数量应为10"
    assert traj.shape[2] == 3, "坐标维度应为3"
    print("  ✓ 轨迹生成测试通过")
    return traj, times


def test_collision(traj):
    print("\n=== 测试碰撞检测 ===")
    report = check_trajectory_collisions(traj, safe_distance=0.5, check_every_n=5)
    print(f"  有碰撞风险: {report['has_risk']}")
    print(f"  碰撞次数: {report['collision_count']}")
    print(f"  最小间距: {report['min_distance_overall']:.4f} m")
    print("  ✓ 碰撞检测测试通过")


if __name__ == "__main__":
    test_formations()
    traj, times = test_trajectory()
    test_collision(traj)
    print("\n=== 所有测试通过 ===")
