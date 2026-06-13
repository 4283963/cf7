package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.TrajectoryMission;
import com.dronecamp.formation.service.PythonTrajectoryClient;
import com.dronecamp.formation.service.TrajectoryService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/api/v1/trajectory")
@CrossOrigin(origins = "*")
public class TrajectoryController {

    @Autowired
    private TrajectoryService trajectoryService;

    @Autowired
    private PythonTrajectoryClient pythonClient;

    @PostMapping("/missions")
    public ApiResponse<TrajectoryMission> createMission(@RequestBody TrajectoryComputeRequest request) {
        try {
            TrajectoryMission mission = trajectoryService.createAndStoreMission(request);
            if (Boolean.TRUE.equals(mission.getHasCollisionRisk())) {
                return ApiResponse.error("轨迹存在碰撞风险，已拦截。请调整编队参数后重试。")
                    .data(mission);
            }
            return ApiResponse.ok(mission);
        } catch (Exception e) {
            log.error("创建轨迹任务失败: {}", e.getMessage(), e);
            return ApiResponse.error("创建任务失败: " + e.getMessage());
        }
    }

    @GetMapping("/missions")
    public ApiResponse<List<TrajectoryMission>> listMissions() {
        try {
            return ApiResponse.ok(trajectoryService.listAllMissions());
        } catch (Exception e) {
            return ApiResponse.error("获取任务列表失败: " + e.getMessage());
        }
    }

    @GetMapping("/missions/{missionId}")
    public ApiResponse<TrajectoryMission> getMission(@PathVariable String missionId) {
        return trajectoryService.getMission(missionId)
            .map(ApiResponse::ok)
            .orElse(ApiResponse.error("任务不存在: " + missionId));
    }

    @GetMapping("/missions/{missionId}/next")
    public ApiResponse<CommandBatch> getNextCommand(
            @PathVariable String missionId,
            @RequestParam(defaultValue = "false") boolean applyCorrection) {
        try {
            CommandBatch batch = trajectoryService.getNextCommandBatch(missionId, applyCorrection);
            return ApiResponse.ok(batch);
        } catch (Exception e) {
            log.error("获取下一批指令失败: {}", e.getMessage());
            return ApiResponse.error(e.getMessage());
        }
    }

    @GetMapping("/missions/{missionId}/commands/{timestep}")
    public ApiResponse<CommandBatch> getCommandAtTimestep(
            @PathVariable String missionId,
            @PathVariable Integer timestep,
            @RequestParam(defaultValue = "false") boolean applyCorrection) {
        try {
            CommandBatch batch = trajectoryService.getCommandBatchByTimestep(missionId, timestep, applyCorrection);
            return ApiResponse.ok(batch);
        } catch (Exception e) {
            log.error("获取指令失败: {}", e.getMessage());
            return ApiResponse.error(e.getMessage());
        }
    }

    @GetMapping("/missions/{missionId}/commands")
    public ApiResponse<List<CommandBatch>> getCommandRange(
            @PathVariable String missionId,
            @RequestParam(defaultValue = "0") int startStep,
            @RequestParam(required = false) Integer endStep,
            @RequestParam(defaultValue = "false") boolean applyCorrection) {
        try {
            if (endStep == null) {
                TrajectoryMission mission = trajectoryService.getMission(missionId)
                    .orElseThrow(() -> new RuntimeException("任务不存在"));
                endStep = mission.getTotalTimesteps() - 1;
            }
            List<CommandBatch> batches = trajectoryService.getCommandBatchRange(
                missionId, startStep, endStep, applyCorrection);
            return ApiResponse.ok(batches);
        } catch (Exception e) {
            log.error("批量获取指令失败: {}", e.getMessage());
            return ApiResponse.error(e.getMessage());
        }
    }

    @PostMapping("/missions/{missionId}/reset")
    public ApiResponse<String> resetMissionCursor(@PathVariable String missionId) {
        trajectoryService.resetMissionCursor(missionId);
        return ApiResponse.ok("任务游标已重置");
    }

    @PostMapping("/missions/{missionId}/store-points")
    public ApiResponse<String> storeTrajectoryPoints(
            @PathVariable String missionId,
            @RequestBody List<Map<String, Object>> batchData) {
        try {
            trajectoryService.storeTrajectoryPoints(missionId, batchData);
            return ApiResponse.ok("轨迹点已存储");
        } catch (Exception e) {
            log.error("存储轨迹点失败: {}", e.getMessage());
            return ApiResponse.error("存储失败: " + e.getMessage());
        }
    }

    @GetMapping("/python/health")
    public ApiResponse<Map<String, Object>> pythonHealth() {
        return ApiResponse.ok(pythonClient.healthCheck());
    }
}
