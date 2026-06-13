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

@Slf4j
@Service
public class AttitudeCorrectionService {

    public static final double WARNING_THRESHOLD_CM = 20.0;
    public static final double EMERGENCY_THRESHOLD_CM = 50.0;
    public static final double WARNING_THRESHOLD_M = WARNING_THRESHOLD_CM / 100.0;
    public static final double EMERGENCY_THRESHOLD_M = EMERGENCY_THRESHOLD_CM / 100.0;

    @Autowired
    private PythonTrajectoryClient pythonClient;

    @Autowired
    private DroneControlCommandRepository commandRepository;

    @Autowired
    private TrajectoryMissionRepository missionRepository;

    @Autowired
    private DroneRepository droneRepository;

    public AttitudeCorrectionResult computeAndStoreCorrection(AttitudeCorrectionRequest request) {
        Map<String, Object> payload = new HashMap<>();
        payload.put("reference_positions", request.getReferencePositions());
        payload.put("actual_positions", request.getActualPositions());
        payload.put("timestep", request.getTimestep());
        payload.put("dt", request.getDt());
        payload.put("session_id", request.getSessionId() != null
            ? request.getSessionId() : "mission-" + request.getMissionId());
        if (request.getWindVector() != null) {
            payload.put("wind_vector", request.getWindVector());
        }
        if (request.getPidKp() != null) payload.put("pid_kp", request.getPidKp());
        if (request.getPidKi() != null) payload.put("pid_ki", request.getPidKi());
        if (request.getPidKd() != null) payload.put("pid_kd", request.getPidKd());

        Map<String, Object> raw = pythonClient.attitudeCorrect(payload);
        AttitudeCorrectionResult result = parsePythonResponse(raw, request);

        if (Boolean.TRUE.equals(request.getPersist())) {
            persistCommands(request, result, raw);
        }

        logDroneStatusUpdates(result);

        if (Boolean.TRUE.equals(result.getForceLand())) {
            log.error("================================================");
            log.error("  🔴 强制迫降触发! missionId={}, 原因={}",
                request.getMissionId(), result.getForceLandReason());
            log.error("================================================");
        } else if ("PID_CORRECTION_ACTIVE".equals(result.getOverallStatus())) {
            log.warn(" 🟡 PID 纠偏激活: missionId={}, maxDeviation={:.2f}cm",
                request.getMissionId(), result.getMaxDeviation() * 100);
        }

        return result;
    }

    private AttitudeCorrectionResult parsePythonResponse(Map<String, Object> raw,
                                                         AttitudeCorrectionRequest request) {
        AttitudeCorrectionResult r = new AttitudeCorrectionResult();
        r.setMissionId(request.getMissionId());
        r.setTimestep(safeInt(raw.get("timestep"), request.getTimestep()));
        r.setOverallStatus(safeString(raw.get("overall_status"), "UNKNOWN"));
        r.setForceLand(Boolean.TRUE.equals(raw.get("force_land")));
        r.setForceLandReason(safeString(raw.get("force_land_reason"), null));
        if (raw.get("land_velocity_z") != null) {
            r.setLandVelocityZ(safeDouble(raw.get("land_velocity_z"), -2.0));
        }
        r.setMaxDeviation(safeDouble(raw.get("max_deviation"), 0.0));
        r.setAverageDeviation(safeDouble(raw.get("average_deviation"), 0.0));
        r.setRmsDeviation(safeDouble(raw.get("rms_deviation"), 0.0));
        r.setWindVectorInput((List<Double>) raw.get("wind_vector_input"));
        r.setWindCancelVelocity((List<Double>) raw.get("wind_cancel_velocity"));
        r.setWindCancelTorque((List<Double>) raw.get("wind_cancel_torque"));
        r.setEmergencyDrones((List<Map<String, Object>>) raw.get("emergency_drones"));
        r.setWarningDrones((List<Map<String, Object>>) raw.get("warning_drones"));
        r.setRecommendations((List<String>) raw.get("recommendations"));
        r.setRawPidResponse(raw);

        List<List<Double>> velCorr = (List<List<Double>>) raw.get("velocity_corrections");
        List<List<Double>> torCorr = (List<List<Double>>) raw.get("torque_corrections");
        List<Double> distances = (List<Double>) raw.get("distances_per_drone");
        List<List<Double>> refs = request.getReferencePositions();

        List<DroneCorrection> perDrone = new ArrayList<>();
        int n = Math.min(
            Math.min(refs != null ? refs.size() : 0, velCorr != null ? velCorr.size() : 0),
            torCorr != null ? torCorr.size() : 0
        );
        for (int i = 0; i < n; i++) {
            DroneCorrection dc = new DroneCorrection();
            dc.setDroneIndex(i);
            dc.setDroneCode("DRONE-" + String.format("%02d", i));

            if (distances != null && i < distances.size()) {
                dc.setDeviationDistance(distances.get(i));
                double d = distances.get(i);
                if (d >= EMERGENCY_THRESHOLD_M) dc.setStatus("EMERGENCY");
                else if (d >= WARNING_THRESHOLD_M) dc.setStatus("WARNING");
                else dc.setStatus("OK");
            } else {
                dc.setDeviationDistance(0.0);
                dc.setStatus("OK");
            }

            if (refs != null && i < refs.size()) {
                List<Double> ref = refs.get(i);
                if (ref != null && ref.size() >= 3) {
                    dc.setTargetX(ref.get(0));
                    dc.setTargetY(ref.get(1));
                    dc.setTargetZ(ref.get(2));
                }
            }

            if (velCorr != null && i < velCorr.size()) {
                List<Double> v = velCorr.get(i);
                if (v != null && v.size() >= 3) {
                    dc.setVelocityX(v.get(0));
                    dc.setVelocityY(v.get(1));
                    dc.setVelocityZ(v.get(2));
                }
            }
            if (torCorr != null && i < torCorr.size()) {
                List<Double> t = torCorr.get(i);
                if (t != null && t.size() >= 3) {
                    dc.setTorqueRoll(t.get(0));
                    dc.setTorquePitch(t.get(1));
                    dc.setTorqueYaw(t.get(2));
                }
            }

            if (Boolean.TRUE.equals(r.getForceLand())) {
                dc.setVelocityX(0.0);
                dc.setVelocityY(0.0);
                dc.setVelocityZ(r.getLandVelocityZ() != null ? r.getLandVelocityZ() : -2.0);
                dc.setTorqueRoll(0.0);
                dc.setTorquePitch(0.0);
                dc.setTorqueYaw(0.0);
                dc.setStatus("EMERGENCY_LAND");
            }
            perDrone.add(dc);
        }
        r.setPerDroneCorrections(perDrone);
        return r;
    }

    @Transactional
    public List<DroneControlCommand> persistCommands(AttitudeCorrectionRequest request,
                                                      AttitudeCorrectionResult result,
                                                      Map<String, Object> raw) {
        Long missionDbId = null;
        String missionAlias = null;
        if (request.getMissionId() != null) {
            Optional<TrajectoryMission> m = missionRepository.findByMissionId(request.getMissionId());
            if (m.isPresent()) {
                missionDbId = m.get().getId();
                missionAlias = m.get().getMissionName();
            }
        }

        List<DroneControlCommand> saved = new ArrayList<>();
        List<Double> windVec = result.getWindVectorInput();
        Double wx = windVec != null && windVec.size() > 0 ? windVec.get(0) : null;
        Double wy = windVec != null && windVec.size() > 1 ? windVec.get(1) : null;
        Double wz = windVec != null && windVec.size() > 2 ? windVec.get(2) : null;

        for (DroneCorrection dc : result.getPerDroneCorrections()) {
            DroneControlCommand cmd = new DroneControlCommand();
            cmd.setMissionId(missionDbId);
            cmd.setMissionAlias(missionAlias);
            cmd.setDroneIndex(dc.getDroneIndex());
            cmd.setDroneCode(dc.getDroneCode());
            cmd.setTimestep(result.getTimestep());
            cmd.setCommandType(Boolean.TRUE.equals(result.getForceLand())
                ? "EMERGENCY_LAND" : "PID_CORRECTION");
            cmd.setSource("PID_WIND_CORRECTION");
            cmd.setTargetX(dc.getTargetX());
            cmd.setTargetY(dc.getTargetY());
            cmd.setTargetZ(dc.getTargetZ());
            cmd.setVelocityX(dc.getVelocityX());
            cmd.setVelocityY(dc.getVelocityY());
            cmd.setVelocityZ(dc.getVelocityZ());
            cmd.setPidCorrectionVx(dc.getVelocityX());
            cmd.setPidCorrectionVy(dc.getVelocityY());
            cmd.setPidCorrectionVz(dc.getVelocityZ());
            cmd.setTorqueRoll(dc.getTorqueRoll());
            cmd.setTorquePitch(dc.getTorquePitch());
            cmd.setTorqueYaw(dc.getTorqueYaw());
            cmd.setWindX(wx);
            cmd.setWindY(wy);
            cmd.setWindZ(wz);
            cmd.setDeviationDistance(dc.getDeviationDistance());
            cmd.setEmergencyLand(Boolean.TRUE.equals(result.getForceLand()));
            cmd.setStatus(dc.getStatus());
            cmd.setIssuedAt(LocalDateTime.now());
            saved.add(commandRepository.save(cmd));
        }
        log.info("已持久化 {} 条控制指令 (mission={}, timestep={}, emergencyLand={})",
            saved.size(), request.getMissionId(), result.getTimestep(), result.getForceLand());
        return saved;
    }

    private void logDroneStatusUpdates(AttitudeCorrectionResult result) {
        for (DroneCorrection dc : result.getPerDroneCorrections()) {
            try {
                Optional<Drone> opt = droneRepository.findByDroneIndex(dc.getDroneIndex());
                Drone drone = opt.orElseGet(() -> {
                    Drone d = new Drone();
                    d.setDroneIndex(dc.getDroneIndex());
                    d.setDroneCode(dc.getDroneCode());
                    return d;
                });
                if ("EMERGENCY".equals(dc.getStatus()) || "EMERGENCY_LAND".equals(dc.getStatus())) {
                    drone.setStatus("EMERGENCY_LAND");
                } else if ("WARNING".equals(dc.getStatus())) {
                    if (!"EMERGENCY_LAND".equals(drone.getStatus())) {
                        drone.setStatus("PID_CORRECTING");
                    }
                } else {
                    if (!"EMERGENCY_LAND".equals(drone.getStatus()) &&
                        !"EMERGENCY".equals(drone.getStatus())) {
                        drone.setStatus("FLYING");
                    }
                }
                drone.setLastUpdateTime(LocalDateTime.now());
                droneRepository.save(drone);
            } catch (Exception e) {
                log.warn("更新无人机 {} 状态失败: {}", dc.getDroneIndex(), e.getMessage());
            }
        }
    }

    public List<DroneControlCommand> getCommandsForMission(String missionId) {
        Optional<TrajectoryMission> m = missionRepository.findByMissionId(missionId);
        if (m.isEmpty()) return Collections.emptyList();
        return commandRepository.findByMissionIdOrderByTimestepDesc(m.get().getId());
    }

    public List<DroneControlCommand> getCommandsForMissionAndDrone(String missionId, Integer droneIndex) {
        Optional<TrajectoryMission> m = missionRepository.findByMissionId(missionId);
        if (m.isEmpty()) return Collections.emptyList();
        return commandRepository.findByMissionIdAndDroneIndex(m.get().getId(), droneIndex);
    }

    private static Integer safeInt(Object o, Integer fallback) {
        if (o == null) return fallback;
        if (o instanceof Number) return ((Number) o).intValue();
        try { return Integer.parseInt(o.toString()); } catch (Exception e) { return fallback; }
    }

    private static Double safeDouble(Object o, Double fallback) {
        if (o == null) return fallback;
        if (o instanceof Number) return ((Number) o).doubleValue();
        try { return Double.parseDouble(o.toString()); } catch (Exception e) { return fallback; }
    }

    private static String safeString(Object o, String fallback) {
        return o != null ? o.toString() : fallback;
    }
}
