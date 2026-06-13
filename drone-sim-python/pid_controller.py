import numpy as np
from collections import deque
from typing import Dict, List, Tuple, Optional


class PIDController:
    def __init__(self, kp: float = 0.6, ki: float = 0.15, kd: float = 0.3,
                 integral_limit: float = 1.0, output_limit: float = 3.0,
                 windup_guard: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.windup_guard = windup_guard
        self._integral = 0.0
        self._prev_error = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def step(self, error: float, dt: float = 0.05) -> float:
        if dt <= 0:
            dt = 0.05

        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        if abs(error) > self.windup_guard:
            self._integral *= 0.5

        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

    def state(self) -> Dict[str, float]:
        return {
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "integral": float(self._integral),
            "prev_error": float(self._prev_error) if self._prev_error is not None else 0.0,
        }


class MultiAxisPID:
    def __init__(self, **pid_kwargs):
        self.x = PIDController(**pid_kwargs)
        self.y = PIDController(**pid_kwargs)
        self.z = PIDController(**pid_kwargs)

    def reset(self):
        self.x.reset()
        self.y.reset()
        self.z.reset()

    def step_3d(self, error_vec: np.ndarray, dt: float = 0.05) -> np.ndarray:
        return np.array([
            self.x.step(float(error_vec[0]), dt),
            self.y.step(float(error_vec[1]), dt),
            self.z.step(float(error_vec[2]), dt),
        ])

    def state(self) -> Dict[str, Dict[str, float]]:
        return {"x": self.x.state(), "y": self.y.state(), "z": self.z.state()}


WIND_STRENGTH_DEFAULT = np.array([0.3, -0.1, 0.02])


def apply_wind_perturbation(
    positions: np.ndarray,
    timestep: int,
    wind_vector: Optional[np.ndarray] = None,
    dt: float = 0.05,
    turbulence: float = 0.05,
) -> np.ndarray:
    num_drones = positions.shape[0]
    if wind_vector is None:
        wind_vector = WIND_STRENGTH_DEFAULT
    wind_vector = np.asarray(wind_vector, dtype=np.float64).reshape(1, 3)

    phase = 2 * np.pi * timestep * dt * 0.7
    periodic = np.array([
        np.sin(phase) * 0.5,
        np.cos(phase * 1.3) * 0.4,
        np.sin(phase * 0.5) * 0.1,
    ]).reshape(1, 3)

    noise = np.random.randn(num_drones, 3) * turbulence
    total_wind = wind_vector + periodic + noise

    displaced = positions + total_wind * dt
    return displaced


def compute_pid_correction(
    reference_positions: np.ndarray,
    actual_positions: np.ndarray,
    timestep: int,
    wind_vector: Optional[np.ndarray] = None,
    pid_kwargs: Optional[Dict] = None,
    dt: float = 0.05,
    per_drone_pids: Optional[Dict[int, MultiAxisPID]] = None,
    max_correction_velocity: float = 2.0,
) -> Dict:
    reference_positions = np.asarray(reference_positions, dtype=np.float64)
    actual_positions = np.asarray(actual_positions, dtype=np.float64)
    num_drones = reference_positions.shape[0]

    if wind_vector is None:
        wind_vector = WIND_STRENGTH_DEFAULT
    wind_vector = np.asarray(wind_vector, dtype=np.float64)

    errors = reference_positions - actual_positions

    per_drone_pids = per_drone_pids or {}
    for i in range(num_drones):
        if i not in per_drone_pids:
            per_drone_pids[i] = MultiAxisPID(
                **(pid_kwargs if pid_kwargs else {}))

    velocity_corrections = np.zeros((num_drones, 3), dtype=np.float64)
    torque_corrections = np.zeros((num_drones, 3), dtype=np.float64)
    pid_states = {}

    for i in range(num_drones):
        error_vec = errors[i]
        correction = per_drone_pids[i].step_3d(error_vec, dt)

        norm = np.linalg.norm(correction)
        if norm > max_correction_velocity:
            correction = correction / norm * max_correction_velocity
        velocity_corrections[i] = correction

        roll = np.clip(correction[1] * 0.3, -0.5, 0.5)
        pitch = np.clip(correction[0] * 0.3, -0.5, 0.5)
        yaw = 0.0
        torque_corrections[i] = np.array([roll, pitch, yaw])

        pid_states[str(i)] = per_drone_pids[i].state()

    wind_cancel = -wind_vector
    wind_cancel_torque = np.array([
        np.clip(-wind_vector[1] * 0.2, -0.3, 0.3),
        np.clip(-wind_vector[0] * 0.2, -0.3, 0.3),
        0.0,
    ])

    distances = np.linalg.norm(errors, axis=1)
    warning_threshold = 0.2
    emergency_threshold = 0.5
    warning_drones = []
    emergency_drones = []
    for i in range(num_drones):
        d = float(distances[i])
        if d >= emergency_threshold:
            emergency_drones.append({"drone_index": i, "distance": d})
        elif d >= warning_threshold:
            warning_drones.append({"drone_index": i, "distance": d})

    overall_status = "OK"
    if emergency_drones:
        overall_status = "EMERGENCY_LAND_REQUIRED"
    elif warning_drones:
        overall_status = "PID_CORRECTION_ACTIVE"

    distances_per_drone = distances.tolist()

    return {
        "timestep": int(timestep),
        "dt": dt,
        "num_drones": int(num_drones),
        "wind_vector_input": wind_vector.tolist(),
        "wind_cancel_velocity": wind_cancel.tolist(),
        "wind_cancel_torque": wind_cancel_torque.tolist(),
        "errors": errors.tolist(),
        "distances_per_drone": distances_per_drone,
        "max_deviation": float(np.max(distances)),
        "average_deviation": float(np.mean(distances)),
        "rms_deviation": float(np.sqrt(np.mean(distances ** 2))),
        "velocity_corrections": velocity_corrections.tolist(),
        "torque_corrections": torque_corrections.tolist(),
        "warning_threshold": warning_threshold,
        "emergency_threshold": emergency_threshold,
        "warning_drones": warning_drones,
        "emergency_drones": emergency_drones,
        "overall_status": overall_status,
        "pid_states": pid_states,
        "recommendations": _generate_recommendations(overall_status, emergency_drones, warning_drones),
    }


def _generate_recommendations(
    overall_status: str,
    emergency_drones: List[Dict],
    warning_drones: List[Dict],
) -> List[str]:
    recs = []
    if overall_status == "EMERGENCY_LAND_REQUIRED":
        recs.append("!!! 立即执行一键迫降 !!!")
        for e in emergency_drones:
            recs.append(f"  - 无人机 {e['drone_index']} 偏离 {e['distance']*100:.1f}cm，超过应急阈值 50cm")
        recs.append("  所有无人机强制垂直下降 2.0m/s，关闭水平推进")
    elif overall_status == "PID_CORRECTION_ACTIVE":
        recs.append("风摆 PID 纠偏已激活")
        for w in warning_drones:
            recs.append(f"  - 无人机 {w['drone_index']} 偏离 {w['distance']*100:.1f}cm（阈值 20cm）")
        recs.append("  反向补偿速度与力矩已下发")
    else:
        recs.append("编队状态正常，PID 待机")
    return recs


class FormationPIDBank:
    def __init__(self, num_drones: int = 10, **pid_kwargs):
        self.num_drones = num_drones
        self.pids: Dict[int, MultiAxisPID] = {}
        for i in range(num_drones):
            self.pids[i] = MultiAxisPID(**pid_kwargs)

    def reset(self):
        for pid in self.pids.values():
            pid.reset()

    def compute(
        self,
        reference_positions: np.ndarray,
        actual_positions: np.ndarray,
        timestep: int,
        wind_vector: Optional[np.ndarray] = None,
        dt: float = 0.05,
    ) -> Dict:
        return compute_pid_correction(
            reference_positions,
            actual_positions,
            timestep,
            wind_vector=wind_vector,
            per_drone_pids=self.pids,
            dt=dt,
        )


def simulate_wind_disturbance_and_correction(
    reference_trajectory: np.ndarray,
    wind_vector: Optional[np.ndarray] = None,
    num_steps: Optional[int] = None,
    dt: float = 0.05,
    turbulence: float = 0.03,
) -> Dict:
    num_drones, total_timesteps, _ = reference_trajectory.shape
    if num_steps is None:
        num_steps = min(60, total_timesteps)

    bank = FormationPIDBank(num_drones=num_drones)

    actual_positions = reference_trajectory[:, 0, :].copy()
    actual_history = np.zeros((num_drones, num_steps, 3))
    corrected_history = np.zeros((num_drones, num_steps, 3))
    deviation_history = np.zeros(num_steps)
    status_history = []

    for t in range(num_steps):
        ref_t = reference_trajectory[:, min(t, total_timesteps - 1), :]
        actual_positions = apply_wind_perturbation(actual_positions, t, wind_vector, dt, turbulence)

        result = bank.compute(ref_t, actual_positions, t, wind_vector=wind_vector, dt=dt)
        vel_corr = np.asarray(result["velocity_corrections"])
        actual_positions = actual_positions + vel_corr * dt

        actual_history[:, t, :] = actual_positions
        corrected_history[:, t, :] = vel_corr
        deviation_history[t] = result["max_deviation"]
        status_history.append(result["overall_status"])

    return {
        "num_steps": num_steps,
        "dt": dt,
        "wind_vector": (wind_vector if wind_vector is not None else WIND_STRENGTH_DEFAULT).tolist(),
        "actual_trajectory": actual_history.tolist(),
        "velocity_corrections": corrected_history.tolist(),
        "max_deviation_history": deviation_history.tolist(),
        "status_history": status_history,
        "final_max_deviation": float(deviation_history[-1]),
        "peak_max_deviation": float(np.max(deviation_history)),
    }
