package com.dronecamp.formation.service;

import com.dronecamp.formation.dto.*;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.*;
import org.springframework.http.client.SimpleClientHttpRequestFactory;
import org.springframework.stereotype.Service;
import org.springframework.core.ParameterizedTypeReference;
import org.springframework.web.client.ResourceAccessException;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

@Slf4j
@Service
public class PythonTrajectoryClient {

    @Value("${python.trajectory.service.url:http://localhost:5001}")
    private String pythonServiceUrl;

    @Value("${python.trajectory.service.timeout-ms:5000}")
    private int timeoutMs;

    @Value("${python.circuit-breaker.failure-threshold:5}")
    private int failureThreshold;

    @Value("${python.circuit-breaker.open-duration-ms:30000}")
    private long openDurationMs;

    private final RestTemplate restTemplate;

    private final AtomicInteger failureCount = new AtomicInteger(0);
    private final AtomicBoolean circuitOpen = new AtomicBoolean(false);
    private final AtomicLong circuitOpenedAt = new AtomicLong(0);
    private final AtomicLong lastSuccessAt = new AtomicLong(0);

    public PythonTrajectoryClient() {
        SimpleClientHttpRequestFactory factory = new SimpleClientHttpRequestFactory();
        factory.setConnectTimeout(3000);
        factory.setReadTimeout(5000);
        this.restTemplate = new RestTemplate(factory);
    }

    private void updateTimeouts() {
        try {
            SimpleClientHttpRequestFactory factory =
                (SimpleClientHttpRequestFactory) restTemplate.getRequestFactory();
            factory.setConnectTimeout(Math.max(1000, timeoutMs / 2));
            factory.setReadTimeout(timeoutMs);
        } catch (Exception e) {
            log.warn("无法动态更新超时配置: {}", e.getMessage());
        }
    }

    private boolean isCircuitOpen() {
        if (!circuitOpen.get()) {
            return false;
        }
        long elapsed = System.currentTimeMillis() - circuitOpenedAt.get();
        if (elapsed >= openDurationMs) {
            log.warn("熔断器半开状态，尝试放行一次请求探测 Python 服务恢复...");
            circuitOpen.set(false);
            failureCount.set(0);
            return false;
        }
        return true;
    }

    private void recordSuccess() {
        failureCount.set(0);
        circuitOpen.set(false);
        lastSuccessAt.set(System.currentTimeMillis());
    }

    private void recordFailure(String reason) {
        int count = failureCount.incrementAndGet();
        log.warn("Python 服务调用失败 (连续第 {} 次): {}", count, reason);
        if (count >= failureThreshold && !circuitOpen.get()) {
            circuitOpen.set(true);
            circuitOpenedAt.set(System.currentTimeMillis());
            log.error("============================================");
            log.error("  ⚡ 熔断器已触发 OPEN 状态");
            log.error("  连续失败: {} 次, 阈值: {} 次", count, failureThreshold);
            log.error("  恢复等待: {} ms (期间所有请求直接降级)", openDurationMs);
            log.error("============================================");
        }
    }

    private <T> T executeWithFallback(SupplierWithException<T> action, FallbackSupplier<T> fallback, String opName) {
        if (isCircuitOpen()) {
            long remaining = openDurationMs - (System.currentTimeMillis() - circuitOpenedAt.get());
            log.warn("熔断器已打开 [{}], 直接降级响应, 剩余等待: {} ms", opName, Math.max(0, remaining));
            return fallback.get("CIRCUIT_OPEN", "熔断器已打开，Python 服务暂时不可用");
        }
        try {
            updateTimeouts();
            T result = action.get();
            recordSuccess();
            return result;
        } catch (ResourceAccessException e) {
            String msg = String.format("Python 服务超时/连接失败 [%s]: %s", opName, e.getMessage());
            log.error(msg);
            recordFailure(msg);
            return fallback.get("TIMEOUT", msg);
        } catch (RestClientException e) {
            String msg = String.format("Python 服务调用异常 [%s]: %s", opName, e.getMessage());
            log.error(msg);
            recordFailure(msg);
            return fallback.get("REST_ERROR", msg);
        } catch (Exception e) {
            String msg = String.format("调用 Python 服务发生未知错误 [%s]: %s", opName, e.getMessage());
            log.error(msg, e);
            recordFailure(msg);
            return fallback.get("UNKNOWN_ERROR", msg);
        }
    }

    @FunctionalInterface
    interface SupplierWithException<T> {
        T get() throws Exception;
    }

    @FunctionalInterface
    interface FallbackSupplier<T> {
        T get(String errorCode, String errorMsg);
    }

    // ==================== 业务方法 ====================

    public Map<String, Object> computeTrajectory(TrajectoryComputeRequest request) {
        String url = pythonServiceUrl + "/api/v1/trajectory/compute";
        log.info("调用 Python 轨迹计算服务: {} (numDrones={})", url, request.getNumDrones());

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<TrajectoryComputeRequest> entity = new HttpEntity<>(request, headers);

        return executeWithFallback(
            () -> {
                ResponseEntity<Map> response = restTemplate.exchange(
                    url, HttpMethod.POST, entity,
                    new ParameterizedTypeReference<Map>() {}
                );
                Map body = response.getBody();
                log.info("Python 轨迹计算成功, trajectory_id={}",
                    body != null ? body.get("trajectory_id") : "N/A");
                return body;
            },
            (code, msg) -> {
                log.error("轨迹计算降级响应: code={}, msg={}", code, msg);
                Map<String, Object> fallback = new HashMap<>();
                fallback.put("trajectory_id", "FALLBACK-" + UUID.randomUUID());
                fallback.put("num_drones", request.getNumDrones() != null ? request.getNumDrones() : 10);
                fallback.put("total_timesteps", 0);
                fallback.put("time_array", Collections.emptyList());
                fallback.put("duration_seconds", 0.0);
                fallback.put("timestep_hz", 0.0);
                fallback.put("has_risk", true);
                Map<String, Object> collisionReport = new HashMap<>();
                collisionReport.put("safe_distance", 0.5);
                collisionReport.put("has_risk", true);
                collisionReport.put("collision_count", 0);
                collisionReport.put("blocked_reason", "PYTHON_SERVICE_UNAVAILABLE");
                collisionReport.put("error_code", code);
                collisionReport.put("error_message", msg);
                fallback.put("collision_report", collisionReport);
                fallback.put("degraded", true);
                fallback.put("degraded_reason", msg);
                return fallback;
            },
            "computeTrajectory"
        );
    }

    public Map<String, Object> getTrajectoryPoint(String trajectoryId, int timestep, boolean applyCorrection) {
        String url = pythonServiceUrl + "/api/v1/trajectory/" + trajectoryId + "/point/" + timestep
            + "?correction=" + applyCorrection;

        return executeWithFallback(
            () -> {
                ResponseEntity<Map> response = restTemplate.getForEntity(url, Map.class);
                return response.getBody();
            },
            (code, msg) -> {
                Map<String, Object> fallback = new HashMap<>();
                fallback.put("trajectory_id", trajectoryId);
                fallback.put("timestep", timestep);
                fallback.put("time_seconds", timestep * 0.05);
                fallback.put("num_drones", 10);
                fallback.put("positions", Collections.emptyList());
                fallback.put("corrections_applied", false);
                fallback.put("correction_vectors", null);
                fallback.put("degraded", true);
                fallback.put("degraded_reason", msg);
                fallback.put("error_code", code);
                return fallback;
            },
            "getTrajectoryPoint-" + timestep
        );
    }

    public Map<String, Object> getTrajectoryBatch(String trajectoryId, TrajectoryBatchRequest batchRequest) {
        String url = pythonServiceUrl + "/api/v1/trajectory/" + trajectoryId + "/batch";
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<TrajectoryBatchRequest> entity = new HttpEntity<>(batchRequest, headers);

        return executeWithFallback(
            () -> {
                ResponseEntity<Map> response = restTemplate.exchange(
                    url, HttpMethod.POST, entity,
                    new ParameterizedTypeReference<Map>() {}
                );
                return response.getBody();
            },
            (code, msg) -> {
                Map<String, Object> fallback = new HashMap<>();
                fallback.put("trajectory_id", trajectoryId);
                fallback.put("num_points", 0);
                fallback.put("risk_count", 0);
                fallback.put("data", Collections.emptyList());
                fallback.put("degraded", true);
                fallback.put("degraded_reason", msg);
                fallback.put("error_code", code);
                return fallback;
            },
            "getTrajectoryBatch-" + batchRequest.getStartStep()
        );
    }

    public Map<String, Object> checkDeviation(Map<String, Object> payload) {
        String url = pythonServiceUrl + "/api/v1/deviation/check";
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<Map> entity = new HttpEntity<>(payload, headers);

        return executeWithFallback(
            () -> {
                ResponseEntity<Map> response = restTemplate.exchange(
                    url, HttpMethod.POST, entity,
                    new ParameterizedTypeReference<Map>() {}
                );
                return response.getBody();
            },
            (code, msg) -> {
                List<List<Double>> actual = (List<List<Double>>) payload.getOrDefault(
                    "actual_positions", Collections.emptyList());
                List<Double> deviations = new ArrayList<>();
                double warningTh = ((Number) payload.getOrDefault("warning_threshold", 0.3)).doubleValue();
                double emergTh = ((Number) payload.getOrDefault("emergency_threshold", 0.8)).doubleValue();
                for (int i = 0; i < actual.size(); i++) {
                    deviations.add(0.0);
                }
                Map<String, Object> fallback = new HashMap<>();
                fallback.put("max_deviation", 0.0);
                fallback.put("average_deviation", 0.0);
                fallback.put("rms_deviation", 0.0);
                fallback.put("warning_threshold", warningTh);
                fallback.put("emergency_threshold", emergTh);
                fallback.put("deviations_per_drone", deviations);
                fallback.put("status", "DEGRADED");
                fallback.put("warnings", Collections.emptyList());
                fallback.put("emergencies", Collections.emptyList());
                fallback.put("recommended_correction_vectors", Collections.emptyList());
                fallback.put("degraded", true);
                fallback.put("degraded_reason", msg);
                fallback.put("error_code", code);
                return fallback;
            },
            "checkDeviation"
        );
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

        return executeWithFallback(
            () -> {
                ResponseEntity<Map> response = restTemplate.exchange(
                    url, HttpMethod.POST, entity,
                    new ParameterizedTypeReference<Map>() {}
                );
                return response.getBody();
            },
            (code, msg) -> {
                Map<String, Object> fallback = new HashMap<>();
                fallback.put("has_collision_risk", true);
                fallback.put("safe_distance", safeDistance);
                fallback.put("num_drones", positions.size());
                fallback.put("collision_details", Collections.emptyList());
                fallback.put("correction_vectors", Collections.emptyList());
                fallback.put("corrected_positions", positions);
                fallback.put("collision_risk_after_correction", true);
                fallback.put("collision_details_after_correction", Collections.emptyList());
                fallback.put("degraded", true);
                fallback.put("degraded_reason", msg);
                fallback.put("error_code", code);
                fallback.put("assume_risk_when_degraded", true);
                return fallback;
            },
            "checkCollision"
        );
    }

    public Map<String, Object> healthCheck() {
        String url = pythonServiceUrl + "/api/v1/health";
        try {
            ResponseEntity<Map> response = restTemplate.getForEntity(url, Map.class);
            recordSuccess();
            Map body = response.getBody();
            if (body == null) body = new HashMap();
            body.put("circuit_status", circuitOpen.get() ? "OPEN" : "CLOSED");
            body.put("consecutive_failures", failureCount.get());
            return body;
        } catch (Exception e) {
            log.warn("Python 服务健康检查失败: {}", e.getMessage());
            recordFailure("healthCheck: " + e.getMessage());
            Map<String, Object> result = new HashMap<>();
            result.put("status", "unreachable");
            result.put("error", e.getMessage());
            result.put("circuit_status", circuitOpen.get() ? "OPEN" : "CLOSED");
            result.put("consecutive_failures", failureCount.get());
            if (circuitOpen.get()) {
                long remaining = openDurationMs - (System.currentTimeMillis() - circuitOpenedAt.get());
                result.put("circuit_retry_after_ms", Math.max(0, remaining));
            }
            return result;
        }
    }

    public Map<String, Object> getCircuitBreakerStatus() {
        Map<String, Object> status = new LinkedHashMap<>();
        status.put("circuit_open", circuitOpen.get());
        status.put("consecutive_failures", failureCount.get());
        status.put("failure_threshold", failureThreshold);
        status.put("open_duration_ms", openDurationMs);
        if (circuitOpen.get()) {
            long openedAt = circuitOpenedAt.get();
            long remaining = openDurationMs - (System.currentTimeMillis() - openedAt);
            status.put("opened_at_timestamp", openedAt);
            status.put("remaining_wait_ms", Math.max(0, remaining));
        }
        long last = lastSuccessAt.get();
        status.put("last_success_at", last > 0 ? new Date(last).toString() : "NEVER");
        status.put("timeout_ms", timeoutMs);
        status.put("service_url", pythonServiceUrl);
        return status;
    }

    public void resetCircuitBreaker() {
        circuitOpen.set(false);
        failureCount.set(0);
        circuitOpenedAt.set(0);
        log.warn("熔断器已被手动重置");
    }
}
