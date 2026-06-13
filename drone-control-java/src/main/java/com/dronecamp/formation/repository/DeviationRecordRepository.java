package com.dronecamp.formation.repository;

import com.dronecamp.formation.entity.DeviationRecord;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import java.util.List;

@Repository
public interface DeviationRecordRepository extends JpaRepository<DeviationRecord, Long> {

    List<DeviationRecord> findByMissionIdOrderByRecordTimeDesc(Long missionId);

    List<DeviationRecord> findByMissionIdAndDroneIndex(Long missionId, Integer droneIndex);

    @Query("SELECT MAX(d.deviationDistance) FROM DeviationRecord d WHERE d.missionId = :missionId")
    Double findMaxDeviationByMissionId(@Param("missionId") Long missionId);

    @Query("SELECT AVG(d.deviationDistance) FROM DeviationRecord d WHERE d.missionId = :missionId")
    Double findAvgDeviationByMissionId(@Param("missionId") Long missionId);

    @Query("SELECT COUNT(d) FROM DeviationRecord d WHERE d.missionId = :missionId AND d.status = 'WARNING'")
    Long countWarningByMissionId(@Param("missionId") Long missionId);

    @Query("SELECT COUNT(d) FROM DeviationRecord d WHERE d.missionId = :missionId AND d.status = 'EMERGENCY'")
    Long countEmergencyByMissionId(@Param("missionId") Long missionId);
}
