package com.dronecamp.formation.dto;

import lombok.Data;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class DroneCommand {
    private Integer droneIndex;
    private String droneCode;
    private Double targetX;
    private Double targetY;
    private Double targetZ;
    private Double velocityX;
    private Double velocityY;
    private Double velocityZ;
}
