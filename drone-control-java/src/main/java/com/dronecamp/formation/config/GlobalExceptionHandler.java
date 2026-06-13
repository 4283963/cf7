package com.dronecamp.formation.config;

import com.dronecamp.formation.dto.ApiResponse;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;
import org.springframework.web.client.ResourceAccessException;
import org.springframework.web.servlet.mvc.method.annotation.ResponseEntityExceptionHandler;

import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeoutException;

@Slf4j
@RestControllerAdvice
public class GlobalExceptionHandler extends ResponseEntityExceptionHandler {

    @ExceptionHandler(RuntimeException.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleRuntimeException(RuntimeException ex) {
        log.error("运行时异常: {}", ex.getMessage(), ex);

        ApiResponse<Map<String, Object>> body;
        HttpStatus status = HttpStatus.INTERNAL_SERVER_ERROR;

        Map<String, Object> detail = new HashMap<>();
        detail.put("timestamp", LocalDateTime.now().toString());
        detail.put("exception_type", ex.getClass().getSimpleName());
        detail.put("message", ex.getMessage());

        Throwable cause = ex.getCause();
        if (cause != null) {
            detail.put("cause", cause.getClass().getSimpleName() + ": " + cause.getMessage());
        }

        if (containsTimeout(ex)) {
            status = HttpStatus.GATEWAY_TIMEOUT;
            detail.put("suggestion", "Python 计算服务响应超时，请检查 Python 服务或降低轨迹复杂度");
            detail.put("retryable", true);
            body = ApiResponse.error("上游 Python 服务超时 (504 Gateway Timeout)");
        } else if (ex.getMessage() != null && (
            ex.getMessage().contains("熔断器") ||
            ex.getMessage().contains("CIRCUIT_OPEN") ||
            ex.getMessage().contains("熔断"))) {
            status = HttpStatus.SERVICE_UNAVAILABLE;
            detail.put("suggestion", "熔断器处于 OPEN 状态，任务已暂停，请稍后重试或手动重置熔断器");
            detail.put("retryable", true);
            body = ApiResponse.error("服务已熔断，暂时不可用 (503 Service Unavailable)");
        } else if (ex.getMessage() != null && ex.getMessage().contains("不存在")) {
            status = HttpStatus.NOT_FOUND;
            body = ApiResponse.error(ex.getMessage());
        } else if (ex.getMessage() != null && ex.getMessage().contains("越界")) {
            status = HttpStatus.BAD_REQUEST;
            body = ApiResponse.error(ex.getMessage());
        } else {
            body = ApiResponse.error("服务器内部错误: " + ex.getMessage());
        }

        body.setData(detail);
        return new ResponseEntity<>(body, status);
    }

    @ExceptionHandler(ResourceAccessException.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleResourceAccess(ResourceAccessException ex) {
        log.error("资源访问异常（连接/超时）: {}", ex.getMessage());

        Map<String, Object> detail = new HashMap<>();
        detail.put("timestamp", LocalDateTime.now().toString());
        detail.put("exception_type", "ResourceAccessException");
        detail.put("message", ex.getMessage());
        detail.put("suggestion", "无法访问 Python 计算服务，请检查服务是否启动以及端口 5001 是否可达");
        detail.put("retryable", true);

        ApiResponse<Map<String, Object>> body = ApiResponse.error("上游 Python 服务连接失败: " + ex.getMessage());
        body.setData(detail);
        return new ResponseEntity<>(body, HttpStatus.GATEWAY_TIMEOUT);
    }

    @ExceptionHandler(TimeoutException.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleTimeout(TimeoutException ex) {
        log.error("超时异常: {}", ex.getMessage());
        Map<String, Object> detail = new HashMap<>();
        detail.put("timestamp", LocalDateTime.now().toString());
        detail.put("suggestion", "请求处理超时，请降低轨迹复杂度后重试");

        ApiResponse<Map<String, Object>> body = ApiResponse.error("请求超时: " + ex.getMessage());
        body.setData(detail);
        return new ResponseEntity<>(body, HttpStatus.GATEWAY_TIMEOUT);
    }

    @ExceptionHandler(IllegalArgumentException.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleIllegalArgument(IllegalArgumentException ex) {
        log.warn("参数错误: {}", ex.getMessage());
        ApiResponse<Map<String, Object>> body = ApiResponse.error("参数错误: " + ex.getMessage());
        return new ResponseEntity<>(body, HttpStatus.BAD_REQUEST);
    }

    @ExceptionHandler(NullPointerException.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleNPE(NullPointerException ex) {
        log.error("空指针异常: {}", ex.getMessage(), ex);
        Map<String, Object> detail = new HashMap<>();
        detail.put("timestamp", LocalDateTime.now().toString());
        detail.put("suggestion", "返回数据结构可能与预期不符，请检查 Python 服务版本");

        ApiResponse<Map<String, Object>> body = ApiResponse.error("内部数据异常，请重试");
        body.setData(detail);
        return new ResponseEntity<>(body, HttpStatus.INTERNAL_SERVER_ERROR);
    }

    @ExceptionHandler(Exception.class)
    public ResponseEntity<ApiResponse<Map<String, Object>>> handleGeneric(Exception ex) {
        log.error("未捕获的异常: {}", ex.getMessage(), ex);
        ApiResponse<Map<String, Object>> body = ApiResponse.error("系统异常: " + ex.getMessage());
        return new ResponseEntity<>(body, HttpStatus.INTERNAL_SERVER_ERROR);
    }

    private boolean containsTimeout(Throwable t) {
        if (t == null) return false;
        String msg = t.getMessage() == null ? "" : t.getMessage().toLowerCase();
        if (msg.contains("timeout") || msg.contains("time out") ||
            msg.contains("超时") || msg.contains("timed out")) {
            return true;
        }
        Throwable cause = t.getCause();
        while (cause != null) {
            if (cause instanceof TimeoutException || cause instanceof ResourceAccessException) {
                return true;
            }
            String causeMsg = cause.getMessage() == null ? "" : cause.getMessage().toLowerCase();
            if (causeMsg.contains("timeout") || causeMsg.contains("超时")) {
                return true;
            }
            cause = cause.getCause();
        }
        return false;
    }
}
