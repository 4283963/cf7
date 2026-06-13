CREATE TABLE IF NOT EXISTS drones (
    id SERIAL PRIMARY KEY,
    drone_code VARCHAR(50) UNIQUE NOT NULL,
    drone_index INTEGER NOT NULL,
    status VARCHAR(30) DEFAULT 'IDLE',
    last_latitude DOUBLE PRECISION,
    last_longitude DOUBLE PRECISION,
    last_altitude DOUBLE PRECISION,
    last_pos_x DOUBLE PRECISION,
    last_pos_y DOUBLE PRECISION,
    last_pos_z DOUBLE PRECISION,
    last_update_time TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS trajectory_missions (
    id SERIAL PRIMARY KEY,
    mission_id VARCHAR(100) UNIQUE NOT NULL,
    mission_name VARCHAR(255) NOT NULL,
    num_drones INTEGER NOT NULL,
    total_timesteps INTEGER NOT NULL,
    duration_seconds DOUBLE PRECISION NOT NULL,
    timestep_hz DOUBLE PRECISION,
    python_trajectory_id VARCHAR(100),
    formation_script TEXT,
    has_collision_risk BOOLEAN DEFAULT FALSE,
    status VARCHAR(30) DEFAULT 'CREATED',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS trajectory_points (
    id SERIAL PRIMARY KEY,
    mission_id BIGINT NOT NULL REFERENCES trajectory_missions(id),
    drone_index INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    time_seconds DOUBLE PRECISION,
    target_x DOUBLE PRECISION NOT NULL,
    target_y DOUBLE PRECISION NOT NULL,
    target_z DOUBLE PRECISION NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_mission_timestep ON trajectory_points(mission_id, timestep);
CREATE INDEX IF NOT EXISTS idx_drone_mission ON trajectory_points(drone_index, mission_id);

CREATE TABLE IF NOT EXISTS deviation_records (
    id SERIAL PRIMARY KEY,
    mission_id BIGINT,
    drone_index INTEGER NOT NULL,
    timestep INTEGER,
    actual_x DOUBLE PRECISION NOT NULL,
    actual_y DOUBLE PRECISION NOT NULL,
    actual_z DOUBLE PRECISION NOT NULL,
    target_x DOUBLE PRECISION,
    target_y DOUBLE PRECISION,
    target_z DOUBLE PRECISION,
    deviation_distance DOUBLE PRECISION NOT NULL,
    status VARCHAR(30),
    record_time TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_mission_deviation ON deviation_records(mission_id);
CREATE INDEX IF NOT EXISTS idx_drone_deviation ON deviation_records(drone_index, mission_id);
CREATE INDEX IF NOT EXISTS idx_timestamp_deviation ON deviation_records(record_time);
