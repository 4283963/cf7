package com.dronecamp.formation.service;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.Drone;
import com.dronecamp.formation.entity.DroneControlCommand;
import com.dronecamp.formation.entity.TrajectoryMission;
import com.dronecamp.formation.repository.DroneControlCommandRepository;
import com.dronecamp.formation.repository.DroneRepository;
import com.dronecamp.formation.repository.TrajectoryMissionRepository;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Service
public class EmergencyLandService {

    public static final String STATUS_EMERGENCY_LAND = "EMERGENCY_LAND";
    public static final String STATUS_GROUNDED = "GROUNDED";

    @Autowired
    private PythonTrajectoryClient pythonClient;

    @Autowired
    private DroneRepository droneRepository;

    @Autowired
    private DroneControlCommandRepository commandRepository;

    @Autowired
    private TrajectoryMissionRepository missionRepository;

    private final Map<String, Boolean> emergencyLandActive = new ConcurrentHashMap<>();
    private volatile boolean globalEmergencyActive = false;

    @Transactional
    public EmergencyLandResult triggerEmergencyLand(EmergencyLandRequest request) {
        log.error("================================================");
        log.error("  🚨🚨🚨 一键迫降被触发!!! 🚨🚨🚨");
        log.error("  原因: {}", request.getReason());
        log.error("  Mission: {}", request.getMissionId());
        log.error("================================================");

        globalEmergencyActive = true;
        if (request.getMissionId() != null) {
            emergencyLandActive.put(request.getMissionId(), true);
        }

        List<Drone> allDrones = droneRepository.findAll();

        List<Double> altitudes = new ArrayList<>();
        for (Drone d : allDrones) {
            altitudes.add(d.getLastPosZ() != null ? d.getLastPosZ() : 2.0);
        }

        Map<String, Object> pyPayload = new HashMap<>();
        pyPayload.put("num_drones", allDrones.size());
        pyPayload.put("current_altitudes", altitudes);
        pyPayload.put("land_velocity_z", request.getLandVelocityZ());
        pyPayload.put("safe_z", request.getSafeZ());

        Map<String, Object> plan = pythonClient.emergencyLandPlan(pyPayload);

        Long missionDbId = null;
        if (request.getMissionId() != null) {
            Optional<TrajectoryMission> m = missionRepository.findByMissionId(request.getMissionId());
            if (m.isPresent()) {
                missionDbId = m.get().getId();
                TrajectoryMission mission = m.get();
                mission.setStatus("EMERGENCY_LANDED");
                missionRepository.save(mission);
            }
        }

        List<Map<String, Object>> perDroneCmds =
            (List<Map<String, Object>>) plan.get("per_drone_commands");

        List<String> statusMessages = new ArrayList<>();
        Integer maxTs = commandRepository.findMaxTimestepByMissionId(missionDbId);
        int ts = (maxTs != null ? maxTs : 0) + 1;

        for (Drone drone : allDrones) {
            int idx = drone.getDroneIndex() != null ? drone.getDroneIndex() : 0;
            Map<String, Object> pc = null;
            if (perDroneCmds != null && idx < perDroneCmds.size()) {
                pc = perDroneCmds.get(idx);
            }

            double velZ = request.getLandVelocityZ();
            double targetZ = request.getSafeZ();
            if (pc != null) {
                if (pc.get("velocity_z") != null) {
                    velZ = ((Number) pc.get("velocity_z")).doubleValue();
                }
                if (pc.get("target_z") != null) {
                    targetZ = ((Number) pc.get("target_z")).doubleValue();
                }
            }

            DroneControlCommand cmd = new DroneControlCommand();
            cmd.setMissionId(missionDbId);
            cmd.setMissionAlias(request.getMissionId());
            cmd.setDroneIndex(idx);
            cmd.setDroneCode(drone.getDroneCode());
            cmd.setTimestep(ts);
            cmd.setCommandType("EMERGENCY_LAND");
            cmd.setSource("MANUAL_TRIGGER_" + request.getReason());
            cmd.setTargetZ(targetZ);
            cmd.setVelocityX(0.0);
            cmd.setVelocityY(0.0);
            cmd.setVelocityZ(velZ);
            cmd.setTorqueRoll(0.0);
            cmd.setTorquePitch(0.0);
            cmd.setTorqueYaw(0.0);
            cmd.setEmergencyLand(true);
            cmd.setStatus(STATUS_EMERGENCY_LAND);
            cmd.setIssuedAt(LocalDateTime.now());
            commandRepository.save(cmd);

            drone.setStatus(STATUS_EMERGENCY_LAND);
            drone.setLastUpdateTime(LocalDateTime.now());
            droneRepository.save(drone);

            statusMessages.add(String.format("DRONE-%02d [%s]: 垂直 %.2f m/s 降落到 %.2fm",
                idx, drone.getDroneCode(), velZ, targetZ));
        }

        EmergencyLandResult result = new EmergencyLandResult();
        result.setMissionId(request.getMissionId());
        result.setMissionDbId(missionDbId);
        result.setAction("FORCE_EMERGENCY_LAND");
        result.setReason(request.getReason());
        result.setTriggeredAt(LocalDateTime.now());
        result.setAffectedDrones(allDrones.size());
        result.setLandVelocityZ(request.getLandVelocityZ());
        result.setAllDronesLanding(true);
        result.setOverallStatus(STATUS_EMERGENCY_LAND);
        result.setDroneLandingStatus(statusMessages);
        result.setRecommendations(Arrays.asList(
            "所有无人机水平推进已关闭",
            "降落架已展开",
            "指示灯切换为红色闪烁",
            "蜂鸣器已开启告警",
            "请勿接近飞行区域",
            "落地后请关闭电源并检查设备状态"
        ));

        log.warn("一键迫降完成: {} 架无人机进入降落模式, missionDbId={}",
            allDrones.size(), missionDbId);
        return result;
    }

    public EmergencyLandResult triggerGlobalEmergencyLand(String reason) {
        EmergencyLandRequest req = new EmergencyLandRequest();
        req.setReason(reason != null ? reason : "GLOBAL_EMERGENCY");
        req.setMissionId(null);
        return triggerEmergencyLand(req);
    }

    public Map<String, Object> getEmergencyStatus() {
        Map<String, Object> status = new LinkedHashMap<>();
        status.put("global_emergency_active", globalEmergencyActive);
        status.put("missions_in_emergency", new ArrayList<>(emergencyLandActive.keySet()));

        long emergencyDrones = droneRepository.findAll().stream()
            .filter(d -> STATUS_EMERGENCY_LAND.equals(d.getStatus()) || "EMERGENCY".equals(d.getStatus()))
            .count();
        status.put("drones_in_emergency", emergencyDrones);
        status.put("total_registered_drones", droneRepository.count());

        List<Map<String, Object>> droneStatus = new ArrayList<>();
        for (Drone d : droneRepository.findAll()) {
            Map<String, Object> ds = new LinkedHashMap<>();
            ds.put("drone_index", d.getDroneIndex());
            ds.put("drone_code", d.getDroneCode());
            ds.put("status", d.getStatus());
            ds.put("last_z", d.getLastPosZ());
            ds.put("last_update", d.getLastUpdateTime() != null ? d.getLastUpdateTime().toString() : null);
            boolean needRed = STATUS_EMERGENCY_LAND.equals(d.getStatus())
                || "EMERGENCY".equals(d.getStatus());
            ds.put("frontend_red_alert", needRed);
            droneStatus.add(ds);
        }
        status.put("per_drone_status", droneStatus);
        status.put("any_drone_need_red_alert",
            droneStatus.stream().anyMatch(ds -> Boolean.TRUE.equals(ds.get("frontend_red_alert"))));

        return status;
    }

    @Transactional
    public String resetEmergencyStatus(String missionId) {
        log.warn("重置紧急状态: missionId={}", missionId);

        if (missionId == null) {
            globalEmergencyActive = false;
            emergencyLandActive.clear();
            for (Drone d : droneRepository.findAll()) {
                if (STATUS_EMERGENCY_LAND.equals(d.getStatus()) || "EMERGENCY".equals(d.getStatus())) {
                    d.setStatus("IDLE");
                    d.setLastUpdateTime(LocalDateTime.now());
                    droneRepository.save(d);
                }
            }
            return "已重置全局紧急状态，所有无人机状态恢复为 IDLE";
        }

        emergencyLandActive.remove(missionId);
        if (emergencyLandActive.isEmpty()) {
            globalEmergencyActive = false;
        }
        Optional<TrajectoryMission> m = missionRepository.findByMissionId(missionId);
        if (m.isPresent()) {
            TrajectoryMission mission = m.get();
            if ("EMERGENCY_LANDED".equals(mission.getStatus())) {
                mission.setStatus("READY");
                missionRepository.save(mission);
            }
        }
        return "已重置任务紧急状态: " + missionId;
    }

    public boolean isMissionBlocked(String missionId) {
        if (globalEmergencyActive) return true;
        if (missionId == null) return false;
        return Boolean.TRUE.equals(emergencyLandActive.get(missionId));
    }
}
