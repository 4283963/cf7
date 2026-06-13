package com.dronecamp.formation.dto;

import lombok.Data;
import java.util.List;
import java.util.Map;

@Data
public class ApiResponse<T> {
    private Boolean success;
    private String message;
    private T data;
    private List<Map<String, Object>> errors;

    public static <T> ApiResponse<T> ok(T data) {
        ApiResponse<T> r = new ApiResponse<>();
        r.setSuccess(true);
        r.setData(data);
        return r;
    }

    public static <T> ApiResponse<T> error(String message) {
        ApiResponse<T> r = new ApiResponse<>();
        r.setSuccess(false);
        r.setMessage(message);
        return r;
    }
}
