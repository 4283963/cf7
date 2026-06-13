package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;
import java.time.LocalDateTime;

@Data
public class EmergencyLandResult {
    private String missionId;
    private Long missionDbId;
    private String action;
    private String reason;
    private LocalDateTime triggeredAt;
    private Integer affectedDrones;
    private Double landVelocityZ;
    private Boolean allDronesLanding;
    private String overallStatus;
    private List<String> droneLandingStatus;
    private List<String> recommendations;
}
