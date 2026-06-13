package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;

@Data
public class TrajectoryBatchRequest {
    private String missionId;
    private Integer startStep;
    private Integer endStep;
    private Integer step = 1;
    private Boolean applyCorrection = false;
    private Double safeDistance = 0.5;
}
