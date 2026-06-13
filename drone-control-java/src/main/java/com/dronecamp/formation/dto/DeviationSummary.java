package com.dronecamp.formation.dto;

import lombok.Data;
import java.time.LocalDateTime;
import java.util.List;

@Data
public class DeviationSummary {
    private String missionId;
    private Long missionDbId;
    private Integer totalRecords;
    private Double maxDeviation;
    private Double avgDeviation;
    private Long warningCount;
    private Long emergencyCount;
    private String overallStatus;
    private LocalDateTime lastUpdateTime;
    private List<DroneDeviationStat> perDroneStats;
}
