import numpy as np
from typing import List, Dict, Tuple
from formations import SAFE_DISTANCE


def check_collision_pairwise(
    positions: np.ndarray,
    safe_distance: float = SAFE_DISTANCE,
) -> Tuple[bool, List[Dict]]:
    num_drones = positions.shape[0]
    collisions = []
    has_collision = False

    for i in range(num_drones):
        for j in range(i + 1, num_drones):
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist < safe_distance:
                has_collision = True
                collisions.append({
                    "drone_a": i,
                    "drone_b": j,
                    "distance": float(dist),
                    "safe_distance": safe_distance,
                    "position_a": positions[i].tolist(),
                    "position_b": positions[j].tolist(),
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
                if d < min_distances[t]:
                    min_distances[t] = d

    summary = {
        "safe_distance": safe_distance,
        "total_timesteps": int(num_timesteps),
        "checked_timesteps": int(num_timesteps / check_every_n),
        "collision_count": len(all_collisions),
        "collision_timesteps": collision_timesteps,
        "has_risk": len(all_collisions) > 0,
        "min_distance_overall": float(np.min(min_distances)),
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
            if dist < safe_distance and dist > 1e-8:
                repulsion = (safe_distance - dist) / dist * diff * repulsion_gain
                corrections[i] += repulsion
    return corrections
