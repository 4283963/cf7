package com.dronecamp.formation.controller;

import org.springframework.web.bind.annotation.*;
import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;

@RestController
@RequestMapping("/api/v1")
@CrossOrigin(origins = "*")
public class HealthController {

    @GetMapping("/health")
    public Map<String, Object> health() {
        Map<String, Object> result = new HashMap<>();
        result.put("status", "OK");
        result.put("service", "drone-formation-control-backend");
        result.put("timestamp", LocalDateTime.now().toString());
        result.put("version", "1.0.0");
        return result;
    }
}
