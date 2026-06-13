package com.dronecamp.formation.dto;

import lombok.Data;

@Data
public class EmergencyLandRequest {
    private String missionId;
    private String reason = "MANUAL_TRIGGER";
    private Double landVelocityZ = -2.0;
    private Double safeZ = 0.1;
    private Boolean killAllHorizontalThrust = true;
    private Boolean enableLandingGear = true;
    private Boolean ledRedBlink = true;
    private Boolean audioAlert = true;
}
