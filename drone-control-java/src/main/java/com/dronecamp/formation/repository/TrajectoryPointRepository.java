package com.dronecamp.formation.repository;

import com.dronecamp.formation.entity.TrajectoryPoint;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import java.util.List;

@Repository
public interface TrajectoryPointRepository extends JpaRepository<TrajectoryPoint, Long> {

    List<TrajectoryPoint> findByMissionIdAndTimestep(Long missionId, Integer timestep);

    List<TrajectoryPoint> findByMissionIdOrderByTimestepAsc(Long missionId);

    @Query("SELECT tp FROM TrajectoryPoint tp WHERE tp.missionId = :missionId " +
           "AND tp.timestep BETWEEN :startStep AND :endStep ORDER BY tp.timestep ASC, tp.droneIndex ASC")
    List<TrajectoryPoint> findByMissionIdAndTimestepRange(
        @Param("missionId") Long missionId,
        @Param("startStep") Integer startStep,
        @Param("endStep") Integer endStep
    );

    @Query("SELECT tp FROM TrajectoryPoint tp WHERE tp.missionId = :missionId " +
           "AND tp.droneIndex = :droneIndex ORDER BY tp.timestep ASC")
    List<TrajectoryPoint> findByMissionIdAndDroneIndex(
        @Param("missionId") Long missionId,
        @Param("droneIndex") Integer droneIndex
    );
}
