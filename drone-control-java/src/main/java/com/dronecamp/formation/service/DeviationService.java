package com.dronecamp.formation.service;

import com.dronecamp.formation.dto.*;
import com.dronecamp.formation.entity.DeviationRecord;
import com.dronecamp.formation.entity.Drone;
import com.dronecamp.formation.entity.TrajectoryMission;
import com.dronecamp.formation.entity.TrajectoryPoint;
import com.dronecamp.formation.repository.DeviationRecordRepository;
import com.dronecamp.formation.repository.DroneRepository;
import com.dronecamp.formation.repository.TrajectoryMissionRepository;
import com.dronecamp.formation.repository.TrajectoryPointRepository;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.*;
import java.util.stream.Collectors;

@Slf4j
@Service
public class DeviationService {

    @Value("${deviation.warning.threshold:0.3}")
    private double warningThreshold;

    @Value("${deviation.emergency.threshold:0.8}")
    private double emergencyThreshold;

    @Autowired
    private DeviationRecordRepository deviationRepository;

    @Autowired
    private DroneRepository droneRepository;

    @Autowired
    private TrajectoryMissionRepository missionRepository;

    @Autowired
    private TrajectoryPointRepository pointRepository;

    @Autowired
    private PythonTrajectoryClient pythonClient;

    @Transactional
    public List<DeviationRecord> recordBatchDeviations(String missionId, Integer timestep,
                                                        List<DronePositionReport> reports) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        List<TrajectoryPoint> targetPoints = pointRepository
            .findByMissionIdAndTimestep(mission.getId(), timestep);

        Map<Integer, TrajectoryPoint> targetMap = targetPoints.stream()
            .collect(Collectors.toMap(TrajectoryPoint::getDroneIndex, tp -> tp));

        List<DeviationRecord> records = new ArrayList<>();
        for (DronePositionReport report : reports) {
            DeviationRecord record = processSingleReport(mission, timestep, report, targetMap);
            records.add(record);
            updateDroneStatus(report, record.getDeviationDistance(), record.getStatus());
        }

        deviationRepository.saveAll(records);
        log.info("记录 {} 条偏离数据, missionId={}, timestep={}", records.size(), missionId, timestep);
        return records;
    }

    private DeviationRecord processSingleReport(TrajectoryMission mission, Integer timestep,
                                                 DronePositionReport report,
                                                 Map<Integer, TrajectoryPoint> targetMap) {
        Integer droneIndex = report.getDroneIndex();
        double actualX = report.getPosX() != null ? report.getPosX() : 0.0;
        double actualY = report.getPosY() != null ? report.getPosY() : 0.0;
        double actualZ = report.getPosZ() != null ? report.getPosZ() : 0.0;

        Double targetX = null, targetY = null, targetZ = null;
        TrajectoryPoint tp = targetMap.get(droneIndex);
        if (tp != null) {
            targetX = tp.getTargetX();
            targetY = tp.getTargetY();
            targetZ = tp.getTargetZ();
        }

        double deviationDistance;
        if (targetX != null) {
            double dx = actualX - targetX;
            double dy = actualY - targetY;
            double dz = actualZ - targetZ;
            deviationDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        } else {
            deviationDistance = 0.0;
        }

        String status;
        if (deviationDistance >= emergencyThreshold) {
            status = "EMERGENCY";
        } else if (deviationDistance >= warningThreshold) {
            status = "WARNING";
        } else {
            status = "OK";
        }

        DeviationRecord record = new DeviationRecord();
        record.setMissionId(mission.getId());
        record.setDroneIndex(droneIndex);
        record.setTimestep(timestep);
        record.setActualX(actualX);
        record.setActualY(actualY);
        record.setActualZ(actualZ);
        record.setTargetX(targetX);
        record.setTargetY(targetY);
        record.setTargetZ(targetZ);
        record.setDeviationDistance(deviationDistance);
        record.setStatus(status);
        record.setRecordTime(LocalDateTime.now());
        return record;
    }

    private void updateDroneStatus(DronePositionReport report, double deviation, String status) {
        Integer droneIndex = report.getDroneIndex();
        if (droneIndex == null) {
            return;
        }

        Optional<Drone> optDrone = droneRepository.findByDroneIndex(droneIndex);
        Drone drone = optDrone.orElseGet(() -> {
            Drone d = new Drone();
            d.setDroneIndex(droneIndex);
            d.setDroneCode(report.getDroneCode() != null ? report.getDroneCode()
                : "DRONE-" + String.format("%02d", droneIndex));
            return d;
        });

        drone.setLastPosX(report.getPosX());
        drone.setLastPosY(report.getPosY());
        drone.setLastPosZ(report.getPosZ());
        drone.setLastLatitude(report.getLatitude());
        drone.setLastLongitude(report.getLongitude());
        drone.setLastAltitude(report.getAltitude());
        drone.setLastUpdateTime(LocalDateTime.now());

        if ("EMERGENCY".equals(status)) {
            drone.setStatus("EMERGENCY");
        } else if ("WARNING".equals(status) && !"EMERGENCY".equals(drone.getStatus())) {
            drone.setStatus("WARNING");
        } else if ("OK".equals(status) && !"EMERGENCY".equals(drone.getStatus())) {
            drone.setStatus("FLYING");
        }

        droneRepository.save(drone);
    }

    public Map<String, Object> checkDeviationWithPython(List<List<Double>> reference,
                                                        List<List<Double>> actual) {
        Map<String, Object> payload = new HashMap<>();
        payload.put("reference_positions", reference);
        payload.put("actual_positions", actual);
        payload.put("warning_threshold", warningThreshold);
        payload.put("emergency_threshold", emergencyThreshold);
        return pythonClient.checkDeviation(payload);
    }

    public DeviationSummary getDeviationSummary(String missionId) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        List<DeviationRecord> allRecords = deviationRepository
            .findByMissionIdOrderByRecordTimeDesc(mission.getId());

        DeviationSummary summary = new DeviationSummary();
        summary.setMissionId(missionId);
        summary.setMissionDbId(mission.getId());
        summary.setTotalRecords((long) allRecords.size());

        Double maxDev = deviationRepository.findMaxDeviationByMissionId(mission.getId());
        Double avgDev = deviationRepository.findAvgDeviationByMissionId(mission.getId());
        Long warnCount = deviationRepository.countWarningByMissionId(mission.getId());
        Long emergCount = deviationRepository.countEmergencyByMissionId(mission.getId());

        summary.setMaxDeviation(maxDev != null ? maxDev : 0.0);
        summary.setAvgDeviation(avgDev != null ? avgDev : 0.0);
        summary.setWarningCount(warnCount != null ? warnCount : 0L);
        summary.setEmergencyCount(emergCount != null ? emergCount : 0L);
        summary.setLastUpdateTime(allRecords.isEmpty() ? null : allRecords.get(0).getRecordTime());

        if (emergCount != null && emergCount > 0) {
            summary.setOverallStatus("EMERGENCY");
        } else if (warnCount != null && warnCount > 0) {
            summary.setOverallStatus("WARNING");
        } else if (allRecords.isEmpty()) {
            summary.setOverallStatus("NO_DATA");
        } else {
            summary.setOverallStatus("OK");
        }

        Map<Integer, List<DeviationRecord>> perDrone = allRecords.stream()
            .collect(Collectors.groupingBy(DeviationRecord::getDroneIndex));

        List<DroneDeviationStat> perDroneStats = new ArrayList<>();
        for (Map.Entry<Integer, List<DeviationRecord>> entry : perDrone.entrySet()) {
            List<DeviationRecord> recs = entry.getValue();
            double max = recs.stream().mapToDouble(DeviationRecord::getDeviationDistance).max().orElse(0);
            double avg = recs.stream().mapToDouble(DeviationRecord::getDeviationDistance).average().orElse(0);
            long w = recs.stream().filter(r -> "WARNING".equals(r.getStatus())).count();
            long e = recs.stream().filter(r -> "EMERGENCY".equals(r.getStatus())).count();
            double last = recs.isEmpty() ? 0 : recs.get(0).getDeviationDistance();

            perDroneStats.add(new DroneDeviationStat(entry.getKey(), max, avg, w, e, last));
        }
        perDroneStats.sort(Comparator.comparing(DroneDeviationStat::getDroneIndex));
        summary.setPerDroneStats(perDroneStats);

        return summary;
    }

    public List<DeviationRecord> getDeviationRecords(String missionId, Integer droneIndex) {
        TrajectoryMission mission = missionRepository.findByMissionId(missionId)
            .orElseThrow(() -> new RuntimeException("任务不存在: " + missionId));

        if (droneIndex != null) {
            return deviationRepository.findByMissionIdAndDroneIndex(mission.getId(), droneIndex);
        }
        return deviationRepository.findByMissionIdOrderByRecordTimeDesc(mission.getId());
    }
}
