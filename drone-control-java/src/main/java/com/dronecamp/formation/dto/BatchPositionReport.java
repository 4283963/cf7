package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;

@Data
public class BatchPositionReport {
    private String missionId;
    private Integer timestep;
    private List<DronePositionReport> positions;
}
