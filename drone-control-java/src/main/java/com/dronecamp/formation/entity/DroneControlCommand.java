package com.dronecamp.formation.entity;

import jakarta.persistence.*;
import lombok.Data;
import java.time.LocalDateTime;

@Data
@Entity
@Table(name = "drone_control_commands", indexes = {
    @Index(name = "idx_control_mission", columnList = "mission_id"),
    @Index(name = "idx_control_drone", columnList = "drone_index, mission_id"),
    @Index(name = "idx_control_timestep", columnList = "timestep"),
    @Index(name = "idx_control_created", columnList = "created_at")
})
public class DroneControlCommand {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "mission_id")
    private Long missionId;

    @Column(name = "mission_alias", length = 100)
    private String missionAlias;

    @Column(name = "drone_index", nullable = false)
    private Integer droneIndex;

    @Column(name = "drone_code", length = 50)
    private String droneCode;

    @Column(name = "timestep")
    private Integer timestep;

    @Column(name = "command_type", length = 50)
    private String commandType;

    @Column(name = "source", length = 50)
    private String source;

    @Column(name = "target_x")
    private Double targetX;

    @Column(name = "target_y")
    private Double targetY;

    @Column(name = "target_z")
    private Double targetZ;

    @Column(name = "velocity_x")
    private Double velocityX;

    @Column(name = "velocity_y")
    private Double velocityY;

    @Column(name = "velocity_z")
    private Double velocityZ;

    @Column(name = "pid_correction_vx")
    private Double pidCorrectionVx;

    @Column(name = "pid_correction_vy")
    private Double pidCorrectionVy;

    @Column(name = "pid_correction_vz")
    private Double pidCorrectionVz;

    @Column(name = "torque_roll")
    private Double torqueRoll;

    @Column(name = "torque_pitch")
    private Double torquePitch;

    @Column(name = "torque_yaw")
    private Double torqueYaw;

    @Column(name = "wind_x")
    private Double windX;

    @Column(name = "wind_y")
    private Double windY;

    @Column(name = "wind_z")
    private Double windZ;

    @Column(name = "deviation_distance")
    private Double deviationDistance;

    @Column(name = "emergency_land")
    private Boolean emergencyLand = false;

    @Column(name = "status", length = 30)
    private String status;

    @Column(name = "issued_at")
    private LocalDateTime issuedAt;

    @Column(name = "created_at")
    private LocalDateTime createdAt = LocalDateTime.now();
}
