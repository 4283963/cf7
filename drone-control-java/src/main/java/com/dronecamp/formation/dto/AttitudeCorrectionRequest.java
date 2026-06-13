package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;
import java.util.Map;

@Data
public class AttitudeCorrectionRequest {
    private String missionId;
    private String sessionId;
    private Integer timestep = 0;
    private Double dt = 0.05;
    private List<List<Double>> referencePositions;
    private List<List<Double>> actualPositions;
    private List<Double> windVector;
    private Double pidKp;
    private Double pidKi;
    private Double pidKd;
    private Boolean enableWindCancel = true;
    private Boolean persist = true;
}
