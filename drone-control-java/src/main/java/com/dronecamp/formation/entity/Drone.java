package com.dronecamp.formation.entity;

import jakarta.persistence.*;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;
import java.time.LocalDateTime;

@Data
@Entity
@Table(name = "drones")
@NoArgsConstructor
@AllArgsConstructor
public class Drone {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(unique = true, nullable = false)
    private String droneCode;

    @Column(nullable = false)
    private Integer droneIndex;

    @Column(nullable = false)
    private String status;

    private Double lastLatitude;
    private Double lastLongitude;
    private Double lastAltitude;

    private Double lastPosX;
    private Double lastPosY;
    private Double lastPosZ;

    private LocalDateTime lastUpdateTime;

    @Column(name = "created_at")
    private LocalDateTime createdAt;

    @PrePersist
    protected void onCreate() {
        createdAt = LocalDateTime.now();
        lastUpdateTime = LocalDateTime.now();
        if (status == null) {
            status = "IDLE";
        }
    }
}
