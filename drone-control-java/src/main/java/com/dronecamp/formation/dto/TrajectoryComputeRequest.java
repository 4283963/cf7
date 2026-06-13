package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;
import java.util.Map;

@Data
public class TrajectoryComputeRequest {
    private String missionName;
    private Integer numDrones = 10;
    private Integer samplesPerSegment = 30;
    private Double safeDistance = 0.5;
    private Boolean generateCharts = false;
    private List<Map<String, Object>> blocks;
    private List<Map<String, Object>> waypoints;
}
