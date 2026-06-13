package com.dronecamp.formation.dto;

import lombok.Data;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class DroneDeviationStat {
    private Integer droneIndex;
    private Double maxDeviation;
    private Double avgDeviation;
    private Long warningCount;
    private Long emergencyCount;
    private Double lastDeviation;
}
