import numpy as np
from typing import List, Dict, Tuple
from formations import SAFE_DISTANCE

MAX_CORRECTION_MAGNITUDE = 0.5
OVERLAP_PERTURBATION_SCALE = 0.1
DISTANCE_FLOOR = 1e-4


def _safe_distance(dist: float) -> float:
    return max(dist, DISTANCE_FLOOR)


def _random_direction(dim: int = 3) -> np.ndarray:
    vec = np.random.randn(dim)
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        vec = np.array([1.0, 0.0, 0.0])
    else:
        vec = vec / norm
    return vec


def check_collision_pairwise(
    positions: np.ndarray,
    safe_distance: float = SAFE_DISTANCE,
) -> Tuple[bool, List[Dict]]:
    num_drones = positions.shape[0]
    collisions = []
    has_collision = False

    for i in range(num_drones):
        for j in range(i + 1, num_drones):
            diff = positions[i] - positions[j]
            dist = np.linalg.norm(diff)
            if np.isnan(dist) or np.isinf(dist):
                dist = 0.0
            if dist < safe_distance:
                has_collision = True
                is_overlap = dist < DISTANCE_FLOOR
                collisions.append({
                    "drone_a": i,
                    "drone_b": j,
                    "distance": float(dist),
                    "safe_distance": safe_distance,
                    "position_a": positions[i].tolist(),
                    "position_b": positions[j].tolist(),
                    "is_overlap": is_overlap,
                })
    return has_collision, collisions


def check_trajectory_collisions(
    trajectory_tensor: np.ndarray,
    safe_distance: float = SAFE_DISTANCE,
    check_every_n: int = 1,
) -> Dict:
    num_drones, num_timesteps, _ = trajectory_tensor.shape
    all_collisions = []
    collision_timesteps = []
    min_distances = np.full((num_timesteps,), np.inf)

    for t in range(0, num_timesteps, check_every_n):
        positions = trajectory_tensor[:, t, :]
        has_col, cols = check_collision_pairwise(positions, safe_distance)
        if has_col:
            collision_timesteps.append(int(t))
            for c in cols:
                c["timestep"] = int(t)
                all_collisions.append(c)

        for i in range(num_drones):
            for j in range(i + 1, num_drones):
                d = np.linalg.norm(positions[i] - positions[j])
                if np.isnan(d) or np.isinf(d):
                    d = 0.0
                if d < min_distances[t]:
                    min_distances[t] = d

    overall_min = float(np.min(min_distances))
    if np.isnan(overall_min) or np.isinf(overall_min):
        overall_min = 0.0

    summary = {
        "safe_distance": safe_distance,
        "total_timesteps": int(num_timesteps),
        "checked_timesteps": int(num_timesteps / check_every_n),
        "collision_count": len(all_collisions),
        "collision_timesteps": collision_timesteps,
        "has_risk": len(all_collisions) > 0,
        "min_distance_overall": overall_min,
        "details": all_collisions,
    }
    return summary


def compute_correction_vectors(
    positions: np.ndarray,
    safe_distance: float = SAFE_DISTANCE,
    repulsion_gain: float = 0.3,
) -> np.ndarray:
    num_drones = positions.shape[0]
    corrections = np.zeros_like(positions)

    for i in range(num_drones):
        for j in range(num_drones):
            if i == j:
                continue
            diff = positions[i] - positions[j]
            dist = np.linalg.norm(diff)

            if np.isnan(dist) or np.isinf(dist):
                dist = 0.0

            if dist >= safe_distance:
                continue

            if dist < DISTANCE_FLOOR:
                direction = _random_direction(3)
                magnitude = safe_distance * repulsion_gain + OVERLAP_PERTURBATION_SCALE
                magnitude = min(magnitude, MAX_CORRECTION_MAGNITUDE)
                corrections[i] += direction * magnitude
            else:
                clamped_dist = _safe_distance(dist)
                repulsion_magnitude = (safe_distance - dist) / clamped_dist * repulsion_gain
                repulsion_magnitude = min(repulsion_magnitude, MAX_CORRECTION_MAGNITUDE)
                unit_diff = diff / clamped_dist
                corrections[i] += unit_diff * repulsion_magnitude

    for i in range(num_drones):
        corr_norm = np.linalg.norm(corrections[i])
        if corr_norm > MAX_CORRECTION_MAGNITUDE:
            corrections[i] = corrections[i] / corr_norm * MAX_CORRECTION_MAGNITUDE
        if not np.all(np.isfinite(corrections[i])):
            corrections[i] = np.zeros(3)

    return corrections
