package com.dronecamp.formation.entity;

import jakarta.persistence.*;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;
import java.time.LocalDateTime;

@Data
@Entity
@Table(name = "trajectory_points", indexes = {
    @Index(name = "idx_mission_timestep", columnList = "missionId, timestep"),
    @Index(name = "idx_drone_mission", columnList = "droneIndex, missionId")
})
@NoArgsConstructor
@AllArgsConstructor
public class TrajectoryPoint {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    private Long missionId;

    @Column(nullable = false)
    private Integer droneIndex;

    @Column(nullable = false)
    private Integer timestep;

    private Double timeSeconds;

    @Column(nullable = false)
    private Double targetX;

    @Column(nullable = false)
    private Double targetY;

    @Column(nullable = false)
    private Double targetZ;

    @Column(name = "created_at")
    private LocalDateTime createdAt;

    @PrePersist
    protected void onCreate() {
        createdAt = LocalDateTime.now();
    }
}
