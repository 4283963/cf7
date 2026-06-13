package com.dronecamp.formation.controller;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.DeviationRecord;
import com.dronecamp.formation.service.DeviationService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;

@Slf4j
@RestController
@RequestMapping("/api/v1/deviation")
@CrossOrigin(origins = "*")
public class DeviationController {

    @Autowired
    private DeviationService deviationService;

    @PostMapping("/report")
    public ApiResponse<List<DeviationRecord>> reportBatchPositions(@RequestBody BatchPositionReport report) {
        try {
            List<DeviationRecord> records = deviationService.recordBatchDeviations(
                report.getMissionId(),
                report.getTimestep(),
                report.getPositions()
            );
            log.info("已记录 {} 条偏离数据, missionId={}", records.size(), report.getMissionId());
            return ApiResponse.ok(records);
        } catch (Exception e) {
            log.error("记录偏离数据失败: {}", e.getMessage(), e);
            return ApiResponse.error("记录失败: " + e.getMessage());
        }
    }

    @PostMapping("/report/single")
    public ApiResponse<DeviationRecord> reportSinglePosition(@RequestBody DronePositionReport report) {
        try {
            BatchPositionReport batch = new BatchPositionReport();
            batch.setMissionId("DEFAULT");
            batch.setTimestep(0);
            batch.setPositions(List.of(report));
            List<DeviationRecord> records = deviationService.recordBatchDeviations(
                batch.getMissionId(), batch.getTimestep(), batch.getPositions());
            if (!records.isEmpty()) {
                return ApiResponse.ok(records.get(0));
            }
            return ApiResponse.error("记录失败");
        } catch (Exception e) {
            log.error("记录单条偏离失败: {}", e.getMessage());
            return ApiResponse.error("记录失败: " + e.getMessage());
        }
    }

    @PostMapping("/check")
    public ApiResponse<Map<String, Object>> checkDeviation(@RequestBody Map<String, Object> payload) {
        try {
            List<List<Double>> reference = (List<List<Double>>) payload.get("reference_positions");
            List<List<Double>> actual = (List<List<Double>>) payload.get("actual_positions");
            Map<String, Object> result = deviationService.checkDeviationWithPython(reference, actual);
            return ApiResponse.ok(result);
        } catch (Exception e) {
            log.error("偏离检查失败: {}", e.getMessage());
            return ApiResponse.error("检查失败: " + e.getMessage());
        }
    }

    @GetMapping("/missions/{missionId}/summary")
    public ApiResponse<DeviationSummary> getSummary(@PathVariable String missionId) {
        try {
            DeviationSummary summary = deviationService.getDeviationSummary(missionId);
            return ApiResponse.ok(summary);
        } catch (Exception e) {
            log.error("获取偏离摘要失败: {}", e.getMessage());
            return ApiResponse.error("获取失败: " + e.getMessage());
        }
    }

    @GetMapping("/missions/{missionId}/records")
    public ApiResponse<List<DeviationRecord>> getRecords(
            @PathVariable String missionId,
            @RequestParam(required = false) Integer droneIndex) {
        try {
            List<DeviationRecord> records = deviationService.getDeviationRecords(missionId, droneIndex);
            return ApiResponse.ok(records);
        } catch (Exception e) {
            log.error("获取偏离记录失败: {}", e.getMessage());
            return ApiResponse.error("获取失败: " + e.getMessage());
        }
    }
}
