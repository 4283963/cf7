import numpy as np


FORMATION_SPACING = 1.0
SAFE_DISTANCE = 0.5


def get_square_grid_formation(num_drones: int = 10) -> np.ndarray:
    positions = np.zeros((num_drones, 3))
    idx = 0
    rows = int(np.ceil(np.sqrt(num_drones)))
    cols = int(np.ceil(num_drones / rows))
    for r in range(rows):
        for c in range(cols):
            if idx >= num_drones:
                break
            x = (c - (cols - 1) / 2.0) * FORMATION_SPACING
            y = (r - (rows - 1) / 2.0) * FORMATION_SPACING
            z = 2.0
            positions[idx] = [x, y, z]
            idx += 1
    return positions


def get_circle_formation(num_drones: int = 10, radius: float = 2.0) -> np.ndarray:
    positions = np.zeros((num_drones, 3))
    angles = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
    for i in range(num_drones):
        positions[i] = [
            radius * np.cos(angles[i]),
            radius * np.sin(angles[i]),
            2.0
        ]
    return positions


def get_v_formation(num_drones: int = 10) -> np.ndarray:
    positions = np.zeros((num_drones, 3))
    for i in range(num_drones):
        side = i % 2
        wing_idx = i // 2
        if i == 0:
            positions[i] = [0.0, 0.0, 2.0]
        else:
            x = wing_idx * FORMATION_SPACING
            y = FORMATION_SPACING * (1 if side == 0 else -1) * (wing_idx)
            z = 2.0
            positions[i] = [x, y, z]
    positions[:, 0] -= positions[:, 0].mean()
    positions[:, 1] -= positions[:, 1].mean()
    return positions


FORMATION_MAP = {
    "square_grid": get_square_grid_formation,
    "circle": get_circle_formation,
    "v": get_v_formation,
}


def get_formation(name: str, num_drones: int = 10) -> np.ndarray:
    fn = FORMATION_MAP.get(name, get_square_grid_formation)
    return fn(num_drones)
