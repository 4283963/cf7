package com.dronecamp.formation.entity;

import jakarta.persistence.*;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;
import java.time.LocalDateTime;

@Data
@Entity
@Table(name = "trajectory_missions")
@NoArgsConstructor
@AllArgsConstructor
public class TrajectoryMission {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(unique = true, nullable = false)
    private String missionId;

    @Column(nullable = false)
    private String missionName;

    @Column(nullable = false)
    private Integer numDrones;

    @Column(nullable = false)
    private Integer totalTimesteps;

    @Column(nullable = false)
    private Double durationSeconds;

    private Double timestepHz;

    @Column(columnDefinition = "TEXT")
    private String pythonTrajectoryId;

    @Column(columnDefinition = "TEXT")
    private String formationScript;

    private Boolean hasCollisionRisk;

    private String status;

    @Column(name = "created_at")
    private LocalDateTime createdAt;

    @PrePersist
    protected void onCreate() {
        createdAt = LocalDateTime.now();
        if (status == null) {
            status = "CREATED";
        }
    }
}
