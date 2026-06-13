import uuid
import time
import math
import threading
from collections import OrderedDict
from flask import Flask, request, jsonify
from flask_cors import CORS
import numpy as np

from formations import get_formation, SAFE_DISTANCE, FORMATION_SPACING
from trajectory import build_trajectory_from_blocks, interpolate_waypoints
from collision import check_trajectory_collisions, check_collision_pairwise, compute_correction_vectors
from visualizer import render_trajectory_3d, render_top_view
from pid_controller import (
    FormationPIDBank,
    compute_pid_correction,
    apply_wind_perturbation,
    simulate_wind_disturbance_and_correction,
    WIND_STRENGTH_DEFAULT,
)


def _sanitize(obj):
    if obj is None:
        return None
    if isinstance(obj, bool):
        return obj
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return 0.0
        return obj
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(item) for item in obj]
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.floating):
        v = float(obj)
        if math.isnan(v) or math.isinf(v):
            return 0.0
        return v
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.ndarray):
        return _sanitize(obj.tolist())
    return obj


app = Flask(__name__)
CORS(app)

trajectory_cache = OrderedDict()
CACHE_MAX_SIZE = 100
_cache_lock = threading.Lock()

pid_bank_cache = OrderedDict()
PID_CACHE_MAX_SIZE = 50
_pid_lock = threading.Lock()


def _cache_put(key: str, value: dict):
    with _cache_lock:
        if key in trajectory_cache:
            del trajectory_cache[key]
        trajectory_cache[key] = value
        trajectory_cache[key]["created_at"] = time.time()
        while len(trajectory_cache) > CACHE_MAX_SIZE:
            trajectory_cache.popitem(last=False)


def _cache_get(key: str) -> dict:
    with _cache_lock:
        if key in trajectory_cache:
            item = trajectory_cache[key]
            return item
        return None


def _tensor_to_list(tensor: np.ndarray) -> list:
    return tensor.tolist()


def _get_pid_bank(session_id: str, num_drones: int = 10) -> FormationPIDBank:
    with _pid_lock:
        if session_id in pid_bank_cache:
            bank = pid_bank_cache[session_id]["bank"]
            pid_bank_cache[session_id]["last_access"] = time.time()
            return bank
        bank = FormationPIDBank(num_drones=num_drones)
        if len(pid_bank_cache) >= PID_CACHE_MAX_SIZE:
            pid_bank_cache.popitem(last=False)
        pid_bank_cache[session_id] = {"bank": bank, "last_access": time.time()}
        return bank


def _reset_pid_bank(session_id: str):
    with _pid_lock:
        if session_id in pid_bank_cache:
            pid_bank_cache[session_id]["bank"].reset()
            pid_bank_cache[session_id]["last_access"] = time.time()
        else:
            pid_bank_cache[session_id] = {
                "bank": FormationPIDBank(num_drones=10),
                "last_access": time.time(),
            }


@app.route("/api/v1/health", methods=["GET"])
def health_check():
    return jsonify({
        "status": "ok",
        "service": "drone-trajectory-simulator",
        "timestamp": time.time(),
    })


@app.route("/api/v1/formations", methods=["GET"])
def list_formations():
    return jsonify({
        "formations": [
            {
                "name": "square_grid",
                "description": "方形网格编队，适合10架无人机为4x3矩阵(最后一行1架)",
                "spacing": FORMATION_SPACING,
            },
            {
                "name": "circle",
                "description": "圆形编队，10架无人机均匀分布在圆周上",
                "spacing": FORMATION_SPACING,
            },
            {
                "name": "v",
                "description": "V字队形，模拟候鸟飞行",
                "spacing": FORMATION_SPACING,
            },
        ],
        "num_drones": 10,
        "safe_distance": SAFE_DISTANCE,
    })


@app.route("/api/v1/formation/<string:name>", methods=["GET"])
def get_formation_endpoint(name: str):
    num_drones = int(request.args.get("num_drones", 10))
    try:
        positions = get_formation(name, num_drones)
        has_col, cols = check_collision_pairwise(positions)
        return jsonify({
            "formation": name,
            "num_drones": num_drones,
            "positions": positions.tolist(),
            "has_collision_risk": has_col,
            "collision_details": cols,
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/v1/trajectory/compute", methods=["POST"])
def compute_trajectory():
    payload = request.get_json(force=True)
    if not payload:
        return jsonify({"error": "请求体不能为空"}), 400

    num_drones = int(payload.get("num_drones", 10))
    samples_per_segment = int(payload.get("samples_per_segment", 30))
    safe_distance = float(payload.get("safe_distance", SAFE_DISTANCE))
    generate_charts = bool(payload.get("generate_charts", False))

    try:
        if "blocks" in payload and isinstance(payload["blocks"], list):
            blocks = payload["blocks"]
            trajectory_tensor, time_array = build_trajectory_from_blocks(
                blocks, num_drones, samples_per_segment
            )
        elif "waypoints" in payload and isinstance(payload["waypoints"], list):
            waypoints = payload["waypoints"]
            trajectory_tensor, time_array = interpolate_waypoints(
                waypoints, num_drones, samples_per_segment
            )
        else:
            return jsonify({
                "error": "必须提供 'blocks' (拖拽代码块) 或 'waypoints' (路径点) 输入"
            }), 400
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": f"轨迹计算失败: {str(e)}"}), 500

    collision_report = check_trajectory_collisions(
        trajectory_tensor, safe_distance, check_every_n=2
    )

    trajectory_id = str(uuid.uuid4())
    cache_payload = {
        "trajectory_tensor": trajectory_tensor,
        "time_array": time_array,
        "collision_report": collision_report,
    }
    _cache_put(trajectory_id, cache_payload)

    dt = float(time_array[1] - time_array[0]) if len(time_array) > 1 else 1.0
    if dt <= 0 or math.isnan(dt) or math.isinf(dt):
        dt = 1.0 / 30.0
    timestep_hz = 1.0 / dt

    response = {
        "trajectory_id": trajectory_id,
        "num_drones": num_drones,
        "total_timesteps": int(trajectory_tensor.shape[1]),
        "time_array": time_array.tolist(),
        "duration_seconds": float(time_array[-1]) if len(time_array) > 0 else 0.0,
        "timestep_hz": timestep_hz,
        "collision_report": collision_report,
    }

    if generate_charts:
        try:
            response["chart_3d_base64"] = render_trajectory_3d(trajectory_tensor)
            response["chart_top_base64"] = render_top_view(trajectory_tensor)
        except Exception as e:
            response["chart_error"] = str(e)

    response["trajectory_preview"] = {}
    for drone_id in range(min(num_drones, 10)):
        response["trajectory_preview"][f"drone_{drone_id}"] = {
            "start": trajectory_tensor[drone_id, 0, :].tolist(),
            "end": trajectory_tensor[drone_id, -1, :].tolist(),
            "sample_points": trajectory_tensor[
                drone_id,
                np.linspace(0, trajectory_tensor.shape[1] - 1, 10, dtype=int),
                :
            ].tolist(),
        }

    return jsonify(_sanitize(response))


@app.route("/api/v1/trajectory/<string:trajectory_id>/point/<int:timestep>", methods=["GET"])
def get_trajectory_point(trajectory_id: str, timestep: int):
    cached = _cache_get(trajectory_id)
    if not cached:
        return jsonify({"error": "轨迹不存在或已过期"}), 404

    trajectory_tensor = cached["trajectory_tensor"]
    time_array = cached["time_array"]

    if timestep < 0 or timestep >= trajectory_tensor.shape[1]:
        return jsonify({
            "error": f"时间步越界: 有效范围 [0, {trajectory_tensor.shape[1] - 1}]"
        }), 400

    apply_correction = request.args.get("correction", "false").lower() == "true"
    positions = trajectory_tensor[:, timestep, :].copy()

    corrections_applied = None
    if apply_correction:
        safe_dist = float(request.args.get("safe_distance", SAFE_DISTANCE))
        has_risk, _ = check_collision_pairwise(positions, safe_dist)
        if has_risk:
            corrections = compute_correction_vectors(positions, safe_dist)
            positions += corrections
            corrections_applied = corrections.tolist()

    response = {
        "trajectory_id": trajectory_id,
        "timestep": timestep,
        "time_seconds": float(time_array[timestep]),
        "num_drones": trajectory_tensor.shape[0],
        "positions": positions.tolist(),
        "corrections_applied": corrections_applied is not None,
        "correction_vectors": corrections_applied,
    }
    return jsonify(_sanitize(response))


@app.route("/api/v1/trajectory/<string:trajectory_id>/batch", methods=["POST"])
def get_trajectory_batch(trajectory_id: str):
    cached = _cache_get(trajectory_id)
    if not cached:
        return jsonify({"error": "轨迹不存在或已过期"}), 404

    payload = request.get_json(force=True, silent=True) or {}
    start_step = int(payload.get("start_step", 0))
    end_step = int(payload.get("end_step", cached["trajectory_tensor"].shape[1] - 1))
    step = int(payload.get("step", 1))
    apply_correction = bool(payload.get("apply_correction", False))
    safe_distance = float(payload.get("safe_distance", SAFE_DISTANCE))

    trajectory_tensor = cached["trajectory_tensor"]
    time_array = cached["time_array"]
    num_timesteps = trajectory_tensor.shape[1]

    start_step = max(0, min(start_step, num_timesteps - 1))
    end_step = max(start_step, min(end_step, num_timesteps - 1))

    timesteps = list(range(start_step, end_step + 1, step))
    batch = []
    risk_flags = []

    for t in timesteps:
        positions = trajectory_tensor[:, t, :].copy()
        has_risk, _ = check_collision_pairwise(positions, safe_distance)
        risk_flags.append(has_risk)
        if apply_correction and has_risk:
            corrections = compute_correction_vectors(positions, safe_distance)
            positions += corrections

        batch.append({
            "timestep": int(t),
            "time_seconds": float(time_array[t]),
            "has_collision_risk": has_risk,
            "positions": positions.tolist(),
        })

    return jsonify(_sanitize({
        "trajectory_id": trajectory_id,
        "num_points": len(batch),
        "risk_count": int(sum(risk_flags)),
        "data": batch,
    }))


@app.route("/api/v1/collision/check", methods=["POST"])
def check_positions_collision():
    payload = request.get_json(force=True)
    if not payload or "positions" not in payload:
        return jsonify({"error": "请求体必须包含 'positions' 字段"}), 400

    positions = np.array(payload["positions"], dtype=np.float64)
    safe_distance = float(payload.get("safe_distance", SAFE_DISTANCE))
    return_correction = bool(payload.get("return_correction", True))

    if positions.ndim != 2 or positions.shape[1] != 3:
        return jsonify({"error": f"positions 形状必须是 (N, 3)，当前为 {positions.shape}"}), 400

    has_col, cols = check_collision_pairwise(positions, safe_distance)
    result = {
        "has_collision_risk": has_col,
        "safe_distance": safe_distance,
        "num_drones": positions.shape[0],
        "collision_details": cols,
    }
    if return_correction:
        corrections = compute_correction_vectors(positions, safe_distance)
        result["correction_vectors"] = corrections.tolist()
        corrected = positions + corrections
        result["corrected_positions"] = corrected.tolist()
        has_col_after, cols_after = check_collision_pairwise(corrected, safe_distance)
        result["collision_risk_after_correction"] = has_col_after
        result["collision_details_after_correction"] = cols_after

    return jsonify(_sanitize(result))


@app.route("/api/v1/deviation/check", methods=["POST"])
def check_deviation():
    payload = request.get_json(force=True)
    if not payload:
        return jsonify({"error": "请求体不能为空"}), 400

    reference = np.array(payload["reference_positions"], dtype=np.float64)
    actual = np.array(payload["actual_positions"], dtype=np.float64)
    threshold = float(payload.get("warning_threshold", 0.3))
    emergency = float(payload.get("emergency_threshold", 0.8))

    if reference.shape != actual.shape:
        return jsonify({
            "error": f"形状不匹配: reference {reference.shape} vs actual {actual.shape}"
        }), 400

    diff = actual - reference
    distances = np.linalg.norm(diff, axis=1)
    max_dev = float(np.max(distances))
    avg_dev = float(np.mean(distances))
    rms_dev = float(np.sqrt(np.mean(distances ** 2)))

    warnings = []
    emergencies = []
    for i, d in enumerate(distances):
        if d >= emergency:
            emergencies.append({
                "drone_id": int(i),
                "deviation": float(d),
                "threshold": emergency,
                "recommended_action": "立即降落或悬停"
            })
        elif d >= threshold:
            warnings.append({
                "drone_id": int(i),
                "deviation": float(d),
                "threshold": threshold,
                "recommended_action": "发送纠正指令"
            })

    correction_vectors = (-diff * 0.4).tolist()

    pid_result = None
    enable_pid = bool(payload.get("enable_pid", False))
    if enable_pid:
        wind = payload.get("wind_vector")
        wind_vec = np.array(wind, dtype=np.float64) if wind else None
        timestep = int(payload.get("timestep", 0))
        dt = float(payload.get("dt", 0.05))
        session_id = payload.get("session_id", "default")
        bank = _get_pid_bank(session_id, num_drones=reference.shape[0])
        pid_result = bank.compute(reference, actual, timestep, wind_vector=wind_vec, dt=dt)

    result = {
        "max_deviation": max_dev,
        "average_deviation": avg_dev,
        "rms_deviation": rms_dev,
        "warning_threshold": threshold,
        "emergency_threshold": emergency,
        "deviations_per_drone": distances.tolist(),
        "status": "EMERGENCY" if emergencies else ("WARNING" if warnings else "OK"),
        "warnings": warnings,
        "emergencies": emergencies,
        "recommended_correction_vectors": correction_vectors,
    }
    if pid_result is not None:
        result["pid_correction"] = pid_result

    return jsonify(_sanitize(result))


@app.route("/api/v1/attitude/correct", methods=["POST"])
def attitude_pid_correction():
    payload = request.get_json(force=True)
    if not payload:
        return jsonify({"error": "请求体不能为空"}), 400

    if "reference_positions" not in payload or "actual_positions" not in payload:
        return jsonify({
            "error": "请求体必须包含 'reference_positions' 和 'actual_positions'"
        }), 400

    reference = np.array(payload["reference_positions"], dtype=np.float64)
    actual = np.array(payload["actual_positions"], dtype=np.float64)
    timestep = int(payload.get("timestep", 0))
    dt = float(payload.get("dt", 0.05))
    session_id = payload.get("session_id", "default_attitude")

    wind = payload.get("wind_vector")
    wind_vec = None
    if wind is not None:
        wind_vec = np.array(wind, dtype=np.float64)
    if wind_vec is None:
        wind_vec = WIND_STRENGTH_DEFAULT.copy()

    kp = payload.get("pid_kp")
    ki = payload.get("pid_ki")
    kd = payload.get("pid_kd")
    pid_kwargs = {}
    if kp is not None:
        pid_kwargs["kp"] = float(kp)
    if ki is not None:
        pid_kwargs["ki"] = float(ki)
    if kd is not None:
        pid_kwargs["kd"] = float(kd)

    num_drones = reference.shape[0]
    if reference.shape != actual.shape:
        return jsonify({
            "error": f"形状不匹配: reference {reference.shape} vs actual {actual.shape}"
        }), 400

    bank = _get_pid_bank(session_id, num_drones=num_drones)
    if pid_kwargs:
        for i in range(num_drones):
            from pid_controller import MultiAxisPID
            bank.pids[i] = MultiAxisPID(**pid_kwargs)

    result = bank.compute(reference, actual, timestep, wind_vector=wind_vec, dt=dt)

    if result["overall_status"] == "EMERGENCY_LAND_REQUIRED":
        result["force_land"] = True
        result["force_land_reason"] = "单架偏离超过应急阈值 50cm"
        result["land_velocity_z"] = -2.0
    else:
        result["force_land"] = False

    return jsonify(_sanitize(result))


@app.route("/api/v1/wind/simulate", methods=["POST"])
def simulate_wind():
    payload = request.get_json(force=True, silent=True) or {}
    positions = np.array(
        payload.get("positions", [[float(i), 0.0, 2.0] for i in range(10)]),
        dtype=np.float64,
    )
    timestep = int(payload.get("timestep", 0))
    dt = float(payload.get("dt", 0.05))
    turbulence = float(payload.get("turbulence", 0.05))
    wind = payload.get("wind_vector")
    wind_vec = np.array(wind, dtype=np.float64) if wind else None

    displaced = apply_wind_perturbation(positions, timestep, wind_vec, dt, turbulence)

    return jsonify(_sanitize({
        "timestep": timestep,
        "dt": dt,
        "wind_vector": (wind_vec if wind_vec is not None else WIND_STRENGTH_DEFAULT).tolist(),
        "turbulence": turbulence,
        "original_positions": positions.tolist(),
        "displaced_positions": displaced.tolist(),
        "displacement_per_drone": (displaced - positions).tolist(),
    }))


@app.route("/api/v1/wind/simulate-full", methods=["POST"])
def simulate_full_wind():
    payload = request.get_json(force=True, silent=True) or {}
    trajectory_id = payload.get("trajectory_id")

    cached = None
    ref_traj = None
    if trajectory_id:
        cached = _cache_get(trajectory_id)
    if cached is not None:
        ref_traj = cached["trajectory_tensor"]
    else:
        blocks = payload.get("blocks")
        waypoints = payload.get("waypoints")
        num_drones = int(payload.get("num_drones", 10))
        samples = int(payload.get("samples_per_segment", 30))
        if blocks:
            ref_traj, _ = build_trajectory_from_blocks(blocks, num_drones, samples)
        elif waypoints:
            ref_traj, _ = interpolate_waypoints(waypoints, num_drones, samples)
        else:
            ref_traj = np.zeros((10, 60, 3))
            for i in range(10):
                for t in range(60):
                    ref_traj[i, t] = [float(i), float(t) * 0.02, 2.0]

    wind = payload.get("wind_vector")
    wind_vec = np.array(wind, dtype=np.float64) if wind else None
    num_steps = payload.get("num_steps")
    dt = float(payload.get("dt", 0.05))
    turbulence = float(payload.get("turbulence", 0.03))

    result = simulate_wind_disturbance_and_correction(
        ref_traj, wind_vector=wind_vec, num_steps=num_steps, dt=dt, turbulence=turbulence)
    return jsonify(_sanitize(result))


@app.route("/api/v1/pid/reset", methods=["POST"])
def reset_pid():
    payload = request.get_json(force=True, silent=True) or {}
    session_id = payload.get("session_id", "default")
    _reset_pid_bank(session_id)
    return jsonify({
        "status": "ok",
        "session_id": session_id,
        "message": "PID 积分项与微分历史已清零",
    })


@app.route("/api/v1/pid/status", methods=["GET"])
def pid_status():
    with _pid_lock:
        sessions = list(pid_bank_cache.keys())
    return jsonify({
        "active_sessions": len(sessions),
        "session_ids": sessions,
        "cache_limit": PID_CACHE_MAX_SIZE,
    })


@app.route("/api/v1/emergency/land-plan", methods=["POST"])
def emergency_land_plan():
    payload = request.get_json(force=True, silent=True) or {}
    num_drones = int(payload.get("num_drones", 10))
    current_altitudes = payload.get("current_altitudes")
    if current_altitudes is None:
        current_altitudes = [2.0] * num_drones
    land_velocity = float(payload.get("land_velocity_z", -2.0))
    safe_z = float(payload.get("safe_z", 0.1))

    commands = []
    for i in range(num_drones):
        z = float(current_altitudes[i]) if i < len(current_altitudes) else 2.0
        steps = int(max(1, abs(z - safe_z) / abs(land_velocity) / 0.05))
        commands.append({
            "drone_index": i,
            "drone_code": f"DRONE-{i:02d}",
            "current_z": z,
            "target_z": safe_z,
            "velocity_z": land_velocity,
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "torque_roll": 0.0,
            "torque_pitch": 0.0,
            "torque_yaw": 0.0,
            "hover_before_land": True,
            "estimated_land_steps": steps,
            "estimated_land_seconds": steps * 0.05,
        })

    return jsonify(_sanitize({
        "action": "FORCE_EMERGENCY_LAND",
        "num_drones": num_drones,
        "land_velocity_z": land_velocity,
        "safe_z": safe_z,
        "global_broadcast": {
            "kill_horizontal": True,
            "enable_landing_gear": True,
            "led_color": "RED_BLINK",
            "audio_alert": True,
        },
        "per_drone_commands": commands,
    }))


if __name__ == "__main__":
    import os
    port = int(os.environ.get("PORT", 5001))
    host = os.environ.get("HOST", "0.0.0.0")
    app.run(host=host, port=port, debug=False, threaded=True)
