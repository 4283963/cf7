package com.dronecamp.formation.dto;

import lombok.Data;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class DroneCorrection {
    private Integer droneIndex;
    private String droneCode;
    private Double deviationDistance;
    private Double targetX;
    private Double targetY;
    private Double targetZ;
    private Double velocityX;
    private Double velocityY;
    private Double velocityZ;
    private Double torqueRoll;
    private Double torquePitch;
    private Double torqueYaw;
    private String status;
}
