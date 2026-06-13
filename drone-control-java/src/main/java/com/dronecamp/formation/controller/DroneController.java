package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.ApiResponse;
import com.dronecamp.formation.entity.Drone;
import com.dronecamp.formation.service.DroneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api/v1/drones")
@CrossOrigin(origins = "*")
public class DroneController {

    @Autowired
    private DroneService droneService;

    @GetMapping
    public ApiResponse<List<Drone>> listAllDrones() {
        return ApiResponse.ok(droneService.listAllDrones());
    }

    @GetMapping("/{index}")
    public ApiResponse<Drone> getDrone(@PathVariable Integer index) {
        return droneService.getDroneByIndex(index)
            .map(ApiResponse::ok)
            .orElse(ApiResponse.error("无人机不存在: index=" + index));
    }

    @PutMapping("/{index}/status")
    public ApiResponse<Drone> updateStatus(
            @PathVariable Integer index,
            @RequestBody Map<String, String> body) {
        String status = body.get("status");
        if (status == null || status.isEmpty()) {
            return ApiResponse.error("status 不能为空");
        }
        try {
            Drone drone = droneService.updateDroneStatus(index, status);
            return ApiResponse.ok(drone);
        } catch (Exception e) {
            return ApiResponse.error(e.getMessage());
        }
    }
}
