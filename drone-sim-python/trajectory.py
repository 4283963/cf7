import numpy as np
from scipy.interpolate import interp1d
from typing import List, Dict, Tuple

OVERLAP_JITTER = 0.02
MIN_DURATION = 0.01


def _deduplicate_positions(positions: np.ndarray, jitter: float = OVERLAP_JITTER) -> np.ndarray:
    n = positions.shape[0]
    for i in range(n):
        for j in range(i + 1, n):
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist < 1e-6:
                perturbation = np.random.randn(3) * jitter
                perturbation[2] = abs(perturbation[2])
                positions[j] = positions[j] + perturbation
    return positions


def compute_min_jerk_trajectory(
    start: np.ndarray,
    end: np.ndarray,
    duration: float,
    num_samples: int = 50,
) -> np.ndarray:
    duration = max(duration, MIN_DURATION)
    t = np.linspace(0, duration, num_samples)
    tau = t / duration
    h = end - start

    pos_traj = start + np.outer(h, (10 * tau**3 - 15 * tau**4 + 6 * tau**5))
    return pos_traj.T


def compute_bezier_trajectory(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    num_samples: int = 50,
) -> np.ndarray:
    t = np.linspace(0, 1, num_samples)
    one_minus_t = 1.0 - t
    B = (
        np.outer(one_minus_t ** 3, p0)
        + np.outer(3 * one_minus_t ** 2 * t, p1)
        + np.outer(3 * one_minus_t * t ** 2, p2)
        + np.outer(t ** 3, p3)
    )
    return B


def interpolate_waypoints(
    waypoints: List[Dict],
    num_drones: int = 10,
    samples_per_segment: int = 30,
) -> Tuple[np.ndarray, np.ndarray]:
    num_segments = len(waypoints) - 1
    if num_segments <= 0:
        raise ValueError("至少需要 2 个路径点")

    formation_name = waypoints[0].get("formation", "square_grid")
    from formations import get_formation
    base_formation = get_formation(formation_name, num_drones)

    total_samples = 1 + num_segments * samples_per_segment
    trajectory_tensor = np.zeros((num_drones, total_samples, 3))
    time_array = np.zeros(total_samples)

    global_offset = np.array(waypoints[0].get("offset", [0.0, 0.0, 0.0]))
    prev_positions = base_formation + global_offset
    trajectory_tensor[:, 0, :] = prev_positions

    current_time = 0.0
    for seg_idx in range(num_segments):
        wp_curr = waypoints[seg_idx]
        wp_next = waypoints[seg_idx + 1]

        form_name = wp_next.get("formation", wp_curr.get("formation", "square_grid"))
        next_formation = get_formation(form_name, num_drones)
        offset = np.array(wp_next.get("offset", [0.0, 0.0, 0.0]))
        duration = float(wp_next.get("duration", 2.0))
        duration = max(duration, MIN_DURATION)

        target_positions = next_formation + offset
        target_positions = _deduplicate_positions(target_positions)

        seg_times = np.linspace(0, duration, samples_per_segment + 1)
        for drone_id in range(num_drones):
            p0 = prev_positions[drone_id]
            p3 = target_positions[drone_id]
            p1 = p0 + (p3 - p0) * 0.33
            p2 = p0 + (p3 - p0) * 0.66
            bezier = compute_bezier_trajectory(p0, p1, p2, p3, samples_per_segment + 1)
            start_col = seg_idx * samples_per_segment
            end_col = start_col + samples_per_segment + 1
            trajectory_tensor[drone_id, start_col:end_col, :] = bezier

        time_segment = current_time + seg_times
        start_t = seg_idx * samples_per_segment
        end_t = start_t + samples_per_segment + 1
        if seg_idx == 0:
            time_array[start_t:end_t] = time_segment
        else:
            time_array[start_t:end_t] = time_segment
        current_time += duration
        prev_positions = target_positions

    return trajectory_tensor, time_array


def build_trajectory_from_blocks(
    blocks: List[Dict],
    num_drones: int = 10,
    samples_per_segment: int = 30,
) -> Tuple[np.ndarray, np.ndarray]:
    waypoints = []
    default_offset = [0.0, 0.0, 0.0]
    waypoints.append({
        "formation": "square_grid",
        "offset": default_offset.copy(),
        "duration": 0.0,
    })

    for block in blocks:
        action = block.get("action", "move")
        params = block.get("params", {})
        duration = params.get("duration", 2.0)
        duration = max(float(duration), MIN_DURATION)
        prev_wp = waypoints[-1]
        curr_offset = prev_wp.get("offset", default_offset.copy()).copy()
        curr_form = prev_wp.get("formation", "square_grid")

        if action == "formation_change":
            curr_form = params.get("formation", curr_form)
        elif action == "move_relative":
            dx = params.get("dx", 0.0)
            dy = params.get("dy", 0.0)
            dz = params.get("dz", 0.0)
            curr_offset[0] += dx
            curr_offset[1] += dy
            curr_offset[2] += dz
        elif action == "move_absolute":
            curr_offset[0] = params.get("x", curr_offset[0])
            curr_offset[1] = params.get("y", curr_offset[1])
            curr_offset[2] = params.get("z", curr_offset[2])
        elif action == "rotate_formation":
            pass

        waypoints.append({
            "formation": curr_form,
            "offset": curr_offset,
            "duration": duration,
        })

    return interpolate_waypoints(waypoints, num_drones, samples_per_segment)
