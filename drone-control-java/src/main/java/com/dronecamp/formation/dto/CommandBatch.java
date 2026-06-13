package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;

@Data
public class CommandBatch {
    private String missionId;
    private Integer timestep;
    private Double timeSeconds;
    private List<DroneCommand> commands;
    private Boolean hasCollisionRisk;
}
