package com.dronecamp.formation.service;

import com.dronecamp.formation.dto.*;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;
import org.springframework.http.*;
import org.springframework.core.ParameterizedTypeReference;

import java.util.*;

@Slf4j
@Service
public class PythonTrajectoryClient {

    @Value("${python.trajectory.service.url:http://localhost:5001}")
    private String pythonServiceUrl;

    private final RestTemplate restTemplate = new RestTemplate();

    public Map<String, Object> computeTrajectory(TrajectoryComputeRequest request) {
        String url = pythonServiceUrl + "/api/v1/trajectory/compute";
        log.info("调用 Python 轨迹计算服务: {}", url);

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<TrajectoryComputeRequest> entity = new HttpEntity<>(request, headers);

        try {
            ResponseEntity<Map> response = restTemplate.exchange(
                url, HttpMethod.POST, entity,
                new ParameterizedTypeReference<Map>() {}
            );
            log.info("Python 轨迹计算成功, trajectory_id={}",
                response.getBody() != null ? response.getBody().get("trajectory_id") : "N/A");
            return response.getBody();
        } catch (Exception e) {
            log.error("调用 Python 轨迹计算服务失败: {}", e.getMessage());
            throw new RuntimeException("Python 轨迹计算服务调用失败: " + e.getMessage(), e);
        }
    }

    public Map<String, Object> getTrajectoryPoint(String trajectoryId, int timestep, boolean applyCorrection) {
        String url = pythonServiceUrl + "/api/v1/trajectory/" + trajectoryId + "/point/" + timestep
            + "?correction=" + applyCorrection;
        try {
            ResponseEntity<Map> response = restTemplate.getForEntity(url, Map.class);
            return response.getBody();
        } catch (Exception e) {
            log.error("获取轨迹点失败: {}", e.getMessage());
            throw new RuntimeException("获取轨迹点失败: " + e.getMessage(), e);
        }
    }

    public Map<String, Object> getTrajectoryBatch(String trajectoryId, TrajectoryBatchRequest batchRequest) {
        String url = pythonServiceUrl + "/api/v1/trajectory/" + trajectoryId + "/batch";
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<TrajectoryBatchRequest> entity = new HttpEntity<>(batchRequest, headers);

        try {
            ResponseEntity<Map> response = restTemplate.exchange(
                url, HttpMethod.POST, entity,
                new ParameterizedTypeReference<Map>() {}
            );
            return response.getBody();
        } catch (Exception e) {
            log.error("批量获取轨迹点失败: {}", e.getMessage());
            throw new RuntimeException("批量获取轨迹点失败: " + e.getMessage(), e);
        }
    }

    public Map<String, Object> checkDeviation(Map<String, Object> payload) {
        String url = pythonServiceUrl + "/api/v1/deviation/check";
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<Map> entity = new HttpEntity<>(payload, headers);

        try {
            ResponseEntity<Map> response = restTemplate.exchange(
                url, HttpMethod.POST, entity,
                new ParameterizedTypeReference<Map>() {}
            );
            return response.getBody();
        } catch (Exception e) {
            log.error("偏离检查失败: {}", e.getMessage());
            throw new RuntimeException("偏离检查失败: " + e.getMessage(), e);
        }
    }

    public Map<String, Object> checkCollision(List<List<Double>> positions, double safeDistance) {
        String url = pythonServiceUrl + "/api/v1/collision/check";
        Map<String, Object> payload = new HashMap<>();
        payload.put("positions", positions);
        payload.put("safe_distance", safeDistance);
        payload.put("return_correction", true);

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<Map> entity = new HttpEntity<>(payload, headers);

        try {
            ResponseEntity<Map> response = restTemplate.exchange(
                url, HttpMethod.POST, entity,
                new ParameterizedTypeReference<Map>() {}
            );
            return response.getBody();
        } catch (Exception e) {
            log.error("碰撞检查失败: {}", e.getMessage());
            throw new RuntimeException("碰撞检查失败: " + e.getMessage(), e);
        }
    }

    public Map<String, Object> healthCheck() {
        String url = pythonServiceUrl + "/api/v1/health";
        try {
            ResponseEntity<Map> response = restTemplate.getForEntity(url, Map.class);
            return response.getBody();
        } catch (Exception e) {
            log.warn("Python 服务健康检查失败: {}", e.getMessage());
            Map<String, Object> result = new HashMap<>();
            result.put("status", "unreachable");
            result.put("error", e.getMessage());
            return result;
        }
    }
}
