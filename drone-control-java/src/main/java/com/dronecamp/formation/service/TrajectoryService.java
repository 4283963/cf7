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
        String pythonTrajectoryId = (String) pythonResult.get("trajectory_id");
        Integer numDrones = (Integer) pythonResult.get("num_drones");
        Integer totalTimesteps = (Integer) pythonResult.get("total_timesteps");
        Double durationSeconds = ((Number) pythonResult.get("duration_seconds")).doubleValue();
        Double timestepHz = pythonResult.containsKey("timestep_hz")
            ? ((Number) pythonResult.get("timestep_hz")).doubleValue() : null;

        Map<String, Object> collisionReport = (Map<String, Object>) pythonResult.get("collision_report");
        Boolean hasCollisionRisk = collisionReport != null
            && Boolean.TRUE.equals(collisionReport.get("has_risk"));

        String missionId = "M-" + System.currentTimeMillis() + "-" +
            UUID.randomUUID().toString().substring(0, 6).toUpperCase();

        TrajectoryMission mission = new TrajectoryMission();
        mission.setMissionId(missionId);
        mission.setMissionName(request.getMissionName() != null ? request.getMissionName() : missionId);
        mission.setNumDrones(numDrones);
        mission.setTotalTimesteps(totalTimesteps);
        mission.setDurationSeconds(durationSeconds);
        mission.setTimestepHz(timestepHz);
        mission.setPythonTrajectoryId(pythonTrajectoryId);
        mission.setHasCollisionRisk(hasCollisionRisk);
        mission.setStatus(hasCollisionRisk ? "BLOCKED" : "READY");
        mission = missionRepository.save(mission);

        log.info("任务已创建: missionId={}, pythonTrajectoryId={}, hasRisk={}",
            missionId, pythonTrajectoryId, hasCollisionRisk);

        missionCursor.put(missionId, 0);
        return mission;
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

        List<Map<String, Object>> data = (List<Map<String, Object>>) pythonResult.get("data");
        if (data == null || data.isEmpty()) {
            throw new RuntimeException("无法从 Python 获取轨迹点");
        }

        Map<String, Object> pointInfo = data.get(0);
        Integer ts = ((Number) pointInfo.get("timestep")).intValue();
        Double timeSeconds = pointInfo.containsKey("time_seconds")
            ? ((Number) pointInfo.get("time_seconds")).doubleValue() : null;
        Boolean hasRisk = (Boolean) pointInfo.get("has_collision_risk");

        List<List<Double>> positions = (List<List<Double>>) pointInfo.get("positions");
        List<DroneCommand> commands = new ArrayList<>();
        for (int i = 0; i < positions.size(); i++) {
            List<Double> pos = positions.get(i);
            DroneCommand cmd = new DroneCommand();
            cmd.setDroneIndex(i);
            cmd.setDroneCode("DRONE-" + String.format("%02d", i));
            cmd.setTargetX(pos.get(0));
            cmd.setTargetY(pos.get(1));
            cmd.setTargetZ(pos.get(2));
            commands.add(cmd);
        }

        CommandBatch batch = new CommandBatch();
        batch.setMissionId(mission.getMissionId());
        batch.setTimestep(ts);
        batch.setTimeSeconds(timeSeconds);
        batch.setCommands(commands);
        batch.setHasCollisionRisk(hasRisk);
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

        List<Map<String, Object>> data = (List<Map<String, Object>>) pythonResult.get("data");
        if (data != null) {
            for (Map<String, Object> pointInfo : data) {
                Integer ts = ((Number) pointInfo.get("timestep")).intValue();
                Double timeSeconds = pointInfo.containsKey("time_seconds")
                    ? ((Number) pointInfo.get("time_seconds")).doubleValue() : null;
                Boolean hasRisk = (Boolean) pointInfo.getOrDefault("has_collision_risk", false);

                List<List<Double>> positions = (List<List<Double>>) pointInfo.get("positions");
                List<DroneCommand> commands = new ArrayList<>();
                for (int i = 0; i < positions.size(); i++) {
                    List<Double> pos = positions.get(i);
                    DroneCommand cmd = new DroneCommand();
                    cmd.setDroneIndex(i);
                    cmd.setDroneCode("DRONE-" + String.format("%02d", i));
                    cmd.setTargetX(pos.get(0));
                    cmd.setTargetY(pos.get(1));
                    cmd.setTargetZ(pos.get(2));
                    commands.add(cmd);
                }

                CommandBatch batch = new CommandBatch();
                batch.setMissionId(missionId);
                batch.setTimestep(ts);
                batch.setTimeSeconds(timeSeconds);
                batch.setCommands(commands);
                batch.setHasCollisionRisk(hasRisk);
                result.add(batch);
            }
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
