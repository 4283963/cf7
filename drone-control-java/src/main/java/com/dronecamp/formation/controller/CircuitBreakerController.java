package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.ApiResponse;
import com.dronecamp.formation.service.PythonTrajectoryClient;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/api/v1/admin/circuit-breaker")
@CrossOrigin(origins = "*")
public class CircuitBreakerController {

    @Autowired
    private PythonTrajectoryClient pythonClient;

    @GetMapping("/status")
    public ApiResponse<Map<String, Object>> getStatus() {
        return ApiResponse.ok(pythonClient.getCircuitBreakerStatus());
    }

    @PostMapping("/reset")
    public ApiResponse<String> reset() {
        pythonClient.resetCircuitBreaker();
        return ApiResponse.ok("熔断器已手动重置");
    }
}
