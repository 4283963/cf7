package com.dronecamp.formation.dto;

import lombok.Data;

@Data
public class DronePositionReport {
    private Integer droneIndex;
    private String droneCode;
    private Double posX;
    private Double posY;
    private Double posZ;
    private Double latitude;
    private Double longitude;
    private Double altitude;
    private Long timestamp;
    private String signalSource;
}
