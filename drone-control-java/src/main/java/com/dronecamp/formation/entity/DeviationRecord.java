package com.dronecamp.formation.entity;

import jakarta.persistence.*;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;
import java.time.LocalDateTime;

@Data
@Entity
@Table(name = "deviation_records", indexes = {
    @Index(name = "idx_mission_deviation", columnList = "missionId"),
    @Index(name = "idx_drone_deviation", columnList = "droneIndex, missionId"),
    @Index(name = "idx_timestamp_deviation", columnList = "recordTime")
})
@NoArgsConstructor
@AllArgsConstructor
public class DeviationRecord {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private Long missionId;

    @Column(nullable = false)
    private Integer droneIndex;

    private Integer timestep;

    @Column(nullable = false)
    private Double actualX;

    @Column(nullable = false)
    private Double actualY;

    @Column(nullable = false)
    private Double actualZ;

    private Double targetX;
    private Double targetY;
    private Double targetZ;

    @Column(nullable = false)
    private Double deviationDistance;

    private String status;

    @Column(name = "record_time")
    private LocalDateTime recordTime;

    @Column(name = "created_at")
    private LocalDateTime createdAt;

    @PrePersist
    protected void onCreate() {
        createdAt = LocalDateTime.now();
        if (recordTime == null) {
            recordTime = LocalDateTime.now();
        }
    }
}
