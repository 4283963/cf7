package com.dronecamp.formation.repository;

import com.dronecamp.formation.entity.DroneControlCommand;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface DroneControlCommandRepository extends JpaRepository<DroneControlCommand, Long> {

    List<DroneControlCommand> findByMissionIdOrderByTimestepDesc(Long missionId);

    List<DroneControlCommand> findByMissionIdAndDroneIndex(Long missionId, Integer droneIndex);

    List<DroneControlCommand> findByMissionIdAndTimestep(Long missionId, Integer timestep);

    @Query("SELECT COUNT(c) FROM DroneControlCommand c WHERE c.missionId = :missionId AND c.emergencyLand = true")
    Long countEmergencyLandCommands(Long missionId);

    @Query("SELECT MAX(c.timestep) FROM DroneControlCommand c WHERE c.missionId = :missionId")
    Integer findMaxTimestepByMissionId(Long missionId);
}
