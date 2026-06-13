package com.dronecamp.formation.service;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.TrajectoryMission;
import com.dronecamp.formation.entity.TrajectoryPoint;
import com.dronecamp.formation.repository.TrajectoryMissionRepository;
import com.dronecamp.formation.repository.TrajectoryPointRepository;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

@Slf4j
@Service
public class TrajectoryService {

    @Autowired
    private TrajectoryMissionRepository missionRepository;

    @Autowired
    private TrajectoryPointRepository pointRepository;

    @Autowired
    private PythonTrajectoryClient pythonClient;

    private final Map<String, Integer> missionCursor = new ConcurrentHashMap<>();

    @Transactional
    public TrajectoryMission createAndStoreMission(TrajectoryComputeRequest request) {
        log.info("创建轨迹任务: missionName={}, numDrones={}",
            request.getMissionName(), request.getNumDrones());

        Map<String, Object> pythonResult = pythonClient.computeTrajectory(request);

        boolean degraded = pythonResult.containsKey("degraded") &&
            Boolean.TRUE.equals(pythonResult.get("degraded"));

        String pythonTrajectoryId = safeString(pythonResult.get("trajectory_id"),
            "FALLBACK-" + java.util.UUID.randomUUID().toString().substring(0, 8));
        Integer numDrones = safeInt(pythonResult.get("num_drones"),
            request.getNumDrones() != null ? request.getNumDrones() : 10);
        Integer totalTimesteps = safeInt(pythonResult.get("total_timesteps"), 0);
        Double durationSeconds = safeDouble(pythonResult.get("duration_seconds"), 0.0);
        Double timestepHz = pythonResult.containsKey("timestep_hz")
            ? safeDouble(pythonResult.get("timestep_hz"), 0.0) : null;

        Map<String, Object> collisionReport = (Map<String, Object>) pythonResult.get("collision_report");
        Boolean hasCollisionRisk = Boolean.TRUE;
        if (collisionReport != null) {
            Object riskFlag = collisionReport.get("has_risk");
            if (riskFlag != null) {
                hasCollisionRisk = Boolean.TRUE.equals(riskFlag);
            }
        }
        if (degraded) {
            hasCollisionRisk = true;
        }

        String missionId = "M-" + System.currentTimeMillis() + "-" +
            java.util.UUID.randomUUID().toString().substring(0, 6).toUpperCase();

        TrajectoryMission mission = new TrajectoryMission();
        mission.setMissionId(missionId);
        mission.setMissionName(request.getMissionName() != null ? request.getMissionName() : missionId);
        mission.setNumDrones(numDrones);
        mission.setTotalTimesteps(totalTimesteps);
        mission.setDurationSeconds(durationSeconds);
        mission.setTimestepHz(timestepHz);
        mission.setPythonTrajectoryId(pythonTrajectoryId);
        mission.setHasCollisionRisk(hasCollisionRisk);

        if (degraded) {
            String reason = pythonResult.get("degraded_reason") != null
                ? pythonResult.get("degraded_reason").toString() : "Python 服务降级";
            mission.setStatus("DEGRADED_BLOCKED");
            mission.setFormationScript("[DEGRADED] " + reason);
            log.warn("任务 {} 创建于降级模式: {} - 默认拦截起飞", missionId, reason);
        } else {
            mission.setStatus(hasCollisionRisk ? "BLOCKED" : "READY");
        }

        mission = missionRepository.save(mission);

        log.info("任务已创建: missionId={}, pythonTrajectoryId={}, hasRisk={}, degraded={}",
            missionId, pythonTrajectoryId, hasCollisionRisk, degraded);

        missionCursor.put(missionId, 0);
        return mission;
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

    @Transactional
    public void storeTrajectoryPoints(String missionId, List<Map<String, Object>> batchData) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        List<TrajectoryPoint> points = new ArrayList<>();
        for (Map<String, Object> pointInfo : batchData) {
            Integer timestep = ((Number) pointInfo.get("timestep")).intValue();
            Double timeSeconds = pointInfo.containsKey("time_seconds")
                ? ((Number) pointInfo.get("time_seconds")).doubleValue() : null;

            List<List<Double>> positions = (List<List<Double>>) pointInfo.get("positions");
            for (int i = 0; i < positions.size(); i++) {
                List<Double> pos = positions.get(i);
                TrajectoryPoint tp = new TrajectoryPoint();
                tp.setMissionId(mission.getId());
                tp.setDroneIndex(i);
                tp.setTimestep(timestep);
                tp.setTimeSeconds(timeSeconds);
                tp.setTargetX(pos.get(0));
                tp.setTargetY(pos.get(1));
                tp.setTargetZ(pos.get(2));
                points.add(tp);
            }
        }

        pointRepository.saveAll(points);
        log.info("已存储 {} 个轨迹点 ({} 个时间步 × {} 架无人机)",
            points.size(), batchData.size(),
            points.size() / Math.max(1, batchData.size()));
    }

    public CommandBatch getNextCommandBatch(String missionId, boolean applyCorrection) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        int cursor = missionCursor.getOrDefault(missionId, 0);
        if (cursor >= mission.getTotalTimesteps()) {
            throw new RuntimeException("任务已完成，没有更多指令");
        }

        CommandBatch batch = getCommandBatchAtTimestep(mission, cursor, applyCorrection);
        missionCursor.put(missionId, cursor + 1);
        return batch;
    }

    public CommandBatch getCommandBatchAtTimestep(TrajectoryMission mission, int timestep, boolean applyCorrection) {
        TrajectoryBatchRequest batchRequest = new TrajectoryBatchRequest();
        batchRequest.setMissionId(mission.getMissionId());
        batchRequest.setStartStep(timestep);
        batchRequest.setEndStep(timestep);
        batchRequest.setApplyCorrection(applyCorrection);

        Map<String, Object> pythonResult = pythonClient.getTrajectoryBatch(
            mission.getPythonTrajectoryId(), batchRequest);

        boolean degraded = pythonResult.containsKey("degraded") &&
            Boolean.TRUE.equals(pythonResult.get("degraded"));

        List<Map<String, Object>> data = (List<Map<String, Object>>) pythonResult.get("data");
        if (data == null || data.isEmpty()) {
            if (degraded) {
                throw new RuntimeException("Python 服务已熔断/降级，无法获取轨迹点，请稍后重试或检查 Python 服务。原因: "
                    + safeString(pythonResult.get("degraded_reason"), "未知"));
            }
            throw new RuntimeException("无法从 Python 获取轨迹点，返回数据为空");
        }

        Map<String, Object> pointInfo = data.get(0);
        Integer ts = safeInt(pointInfo.get("timestep"), timestep);
        Double timeSeconds = pointInfo.containsKey("time_seconds")
            ? safeDouble(pointInfo.get("time_seconds"), timestep * 0.05) : null;
        Boolean hasRisk = pointInfo.containsKey("has_collision_risk")
            ? Boolean.TRUE.equals(pointInfo.get("has_collision_risk")) : Boolean.FALSE;

        List<List<Double>> positions = (List<List<Double>>) pointInfo.get("positions");
        if (positions == null) {
            positions = new ArrayList<>();
        }

        List<DroneCommand> commands = new ArrayList<>();
        for (int i = 0; i < positions.size(); i++) {
            List<Double> pos = positions.get(i);
            if (pos == null || pos.size() < 3) continue;
            DroneCommand cmd = new DroneCommand();
            cmd.setDroneIndex(i);
            cmd.setDroneCode("DRONE-" + String.format("%02d", i));
            cmd.setTargetX(safeDouble(pos.get(0), 0.0));
            cmd.setTargetY(safeDouble(pos.get(1), 0.0));
            cmd.setTargetZ(safeDouble(pos.get(2), 0.0));
            commands.add(cmd);
        }

        CommandBatch batch = new CommandBatch();
        batch.setMissionId(mission.getMissionId());
        batch.setTimestep(ts);
        batch.setTimeSeconds(timeSeconds);
        batch.setCommands(commands);
        batch.setHasCollisionRisk(hasRisk || degraded);
        return batch;
    }

    public CommandBatch getCommandBatchByTimestep(String missionId, int timestep, boolean applyCorrection) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        if (timestep < 0 || timestep >= mission.getTotalTimesteps()) {
            throw new RuntimeException("时间步越界: [0, " + (mission.getTotalTimesteps() - 1) + "]");
        }
        return getCommandBatchAtTimestep(mission, timestep, applyCorrection);
    }

    public List<CommandBatch> getCommandBatchRange(String missionId, int startStep, int endStep, boolean applyCorrection) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        List<CommandBatch> result = new ArrayList<>();
        int start = Math.max(0, startStep);
        int end = Math.min(mission.getTotalTimesteps() - 1, endStep);

        TrajectoryBatchRequest batchRequest = new TrajectoryBatchRequest();
        batchRequest.setMissionId(mission.getMissionId());
        batchRequest.setStartStep(start);
        batchRequest.setEndStep(end);
        batchRequest.setApplyCorrection(applyCorrection);

        Map<String, Object> pythonResult = pythonClient.getTrajectoryBatch(
            mission.getPythonTrajectoryId(), batchRequest);

        boolean degraded = pythonResult.containsKey("degraded") &&
            Boolean.TRUE.equals(pythonResult.get("degraded"));

        List<Map<String, Object>> data = (List<Map<String, Object>>) pythonResult.get("data");
        if (data != null) {
            for (Map<String, Object> pointInfo : data) {
                Integer ts = safeInt(pointInfo.get("timestep"), start);
                Double timeSeconds = pointInfo.containsKey("time_seconds")
                    ? safeDouble(pointInfo.get("time_seconds"), ts * 0.05) : null;
                Boolean hasRisk = pointInfo.containsKey("has_collision_risk")
                    ? Boolean.TRUE.equals(pointInfo.get("has_collision_risk")) : Boolean.FALSE;

                List<List<Double>> positions = (List<List<Double>>) pointInfo.get("positions");
                if (positions == null) positions = new ArrayList<>();

                List<DroneCommand> commands = new ArrayList<>();
                for (int i = 0; i < positions.size(); i++) {
                    List<Double> pos = positions.get(i);
                    if (pos == null || pos.size() < 3) continue;
                    DroneCommand cmd = new DroneCommand();
                    cmd.setDroneIndex(i);
                    cmd.setDroneCode("DRONE-" + String.format("%02d", i));
                    cmd.setTargetX(safeDouble(pos.get(0), 0.0));
                    cmd.setTargetY(safeDouble(pos.get(1), 0.0));
                    cmd.setTargetZ(safeDouble(pos.get(2), 0.0));
                    commands.add(cmd);
                }

                CommandBatch batch = new CommandBatch();
                batch.setMissionId(missionId);
                batch.setTimestep(ts);
                batch.setTimeSeconds(timeSeconds);
                batch.setCommands(commands);
                batch.setHasCollisionRisk(hasRisk || degraded);
                result.add(batch);
            }
        } else if (degraded) {
            throw new RuntimeException("Python 服务已熔断/降级，无法批量获取指令。原因: "
                + safeString(pythonResult.get("degraded_reason"), "未知"));
        }
        return result;
    }

    public void resetMissionCursor(String missionId) {
        missionCursor.put(missionId, 0);
        log.info("任务游标已重置: missionId={}", missionId);
    }

    public Optional<TrajectoryMission> getMission(String missionId) {
        return missionRepository.findByMissionId(missionId);
    }

    public List<TrajectoryMission> listAllMissions() {
        return missionRepository.findAll();
    }
}
