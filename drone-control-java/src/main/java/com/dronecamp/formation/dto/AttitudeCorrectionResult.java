package com.dronecamp.formation.dto;

import lombok.Data;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;
import java.util.List;
import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class AttitudeCorrectionResult {
    private String missionId;
    private Integer timestep;
    private String overallStatus;
    private Boolean forceLand;
    private String forceLandReason;
    private Double landVelocityZ;
    private Double maxDeviation;
    private Double averageDeviation;
    private Double rmsDeviation;
    private List<Double> windVectorInput;
    private List<Double> windCancelVelocity;
    private List<Double> windCancelTorque;
    private List<DroneCorrection> perDroneCorrections;
    private List<Map<String, Object>> emergencyDrones;
    private List<Map<String, Object>> warningDrones;
    private List<String> recommendations;
    private Map<String, Object> rawPidResponse;
}
