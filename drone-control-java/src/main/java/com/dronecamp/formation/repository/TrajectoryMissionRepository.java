package com.dronecamp.formation.repository;

import com.dronecamp.formation.entity.TrajectoryMission;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import java.util.Optional;

@Repository
public interface TrajectoryMissionRepository extends JpaRepository<TrajectoryMission, Long> {
    Optional<TrajectoryMission> findByMissionId(String missionId);
}
