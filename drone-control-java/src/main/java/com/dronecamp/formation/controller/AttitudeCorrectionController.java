package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.DroneControlCommand;
import com.dronecamp.formation.service.AttitudeCorrectionService;
import com.dronecamp.formation.service.EmergencyLandService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/api/v1/attitude")
@CrossOrigin(origins = "*")
public class AttitudeCorrectionController {

    @Autowired
    private AttitudeCorrectionService correctionService;

    @PostMapping("/correct")
    public ApiResponse<AttitudeCorrectionResult> computeCorrection(
            @RequestBody AttitudeCorrectionRequest request) {
        try {
            AttitudeCorrectionResult result =
                correctionService.computeAndStoreCorrection(request);
            if (Boolean.TRUE.equals(result.getForceLand())) {
                log.error("PID 计算后触发强制迫降: missionId={}, reason={}",
                    request.getMissionId(), result.getForceLandReason());
            }
            return ApiResponse.ok(result);
        } catch (Exception e) {
            log.error("姿态纠偏计算失败: {}", e.getMessage(), e);
            return ApiResponse.error("纠偏计算失败: " + e.getMessage());
        }
    }

    @GetMapping("/commands/mission/{missionId}")
    public ApiResponse<List<DroneControlCommand>> getCommands(
            @PathVariable String missionId) {
        try {
            return ApiResponse.ok(correctionService.getCommandsForMission(missionId));
        } catch (Exception e) {
            return ApiResponse.error(e.getMessage());
        }
    }

    @GetMapping("/commands/mission/{missionId}/drone/{droneIndex}")
    public ApiResponse<List<DroneControlCommand>> getDroneCommands(
            @PathVariable String missionId,
            @PathVariable Integer droneIndex) {
        try {
            return ApiResponse.ok(
                correctionService.getCommandsForMissionAndDrone(missionId, droneIndex));
        } catch (Exception e) {
            return ApiResponse.error(e.getMessage());
        }
    }
}
