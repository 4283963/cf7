package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.ApiResponse;
import com.dronecamp.formation.dto.EmergencyLandRequest;
import com.dronecamp.formation.dto.EmergencyLandResult;
import com.dronecamp.formation.service.EmergencyLandService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/api/v1/emergency")
@CrossOrigin(origins = "*")
public class EmergencyController {

    @Autowired
    private EmergencyLandService emergencyLandService;

    @PostMapping("/land")
    public ApiResponse<EmergencyLandResult> triggerEmergencyLand(
            @RequestBody(required = false) EmergencyLandRequest request) {
        if (request == null) {
            request = new EmergencyLandRequest();
            request.setReason("MANUAL_TRIGGER");
        }
        try {
            EmergencyLandResult result = emergencyLandService.triggerEmergencyLand(request);
            return ApiResponse.ok(result);
        } catch (Exception e) {
            log.error("一键迫降执行失败: {}", e.getMessage(), e);
            return ApiResponse.error("迫降执行失败: " + e.getMessage());
        }
    }

    @PostMapping("/land/global")
    public ApiResponse<EmergencyLandResult> triggerGlobalLand(
            @RequestBody(required = false) Map<String, String> body) {
        String reason = body != null ? body.get("reason") : null;
        try {
            EmergencyLandResult result = emergencyLandService.triggerGlobalEmergencyLand(reason);
            return ApiResponse.ok(result);
        } catch (Exception e) {
            log.error("全局一键迫降执行失败: {}", e.getMessage(), e);
            return ApiResponse.error("迫降执行失败: " + e.getMessage());
        }
    }

    @GetMapping("/status")
    public ApiResponse<Map<String, Object>> getStatus() {
        try {
            return ApiResponse.ok(emergencyLandService.getEmergencyStatus());
        } catch (Exception e) {
            return ApiResponse.error(e.getMessage());
        }
    }

    @PostMapping("/reset")
    public ApiResponse<String> resetEmergency(
            @RequestBody(required = false) Map<String, String> body) {
        String missionId = body != null ? body.get("missionId") : null;
        try {
            String msg = emergencyLandService.resetEmergencyStatus(missionId);
            return ApiResponse.ok(msg);
        } catch (Exception e) {
            log.error("重置紧急状态失败: {}", e.getMessage());
            return ApiResponse.error("重置失败: " + e.getMessage());
        }
    }
}
