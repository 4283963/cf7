package com.dronecamp.formation.service;

import com.dronecamp.formation.entity.Drone;
import com.dronecamp.formation.repository.DroneRepository;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import jakarta.annotation.PostConstruct;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Slf4j
@Service
public class DroneService {

    @Autowired
    private DroneRepository droneRepository;

    @PostConstruct
    public void initDefaultDrones() {
        long count = droneRepository.count();
        if (count == 0) {
            log.info("初始化 10 架默认无人机...");
            List<Drone> drones = new ArrayList<>();
            for (int i = 0; i < 10; i++) {
                Drone d = new Drone();
                d.setDroneIndex(i);
                d.setDroneCode("DRONE-" + String.format("%02d", i));
                d.setStatus("IDLE");
                drones.add(d);
            }
            droneRepository.saveAll(drones);
            log.info("已初始化 10 架默认无人机");
        }
    }

    public List<Drone> listAllDrones() {
        return droneRepository.findAll();
    }

    public Optional<Drone> getDroneByIndex(Integer index) {
        return droneRepository.findByDroneIndex(index);
    }

    public Drone updateDroneStatus(Integer index, String status) {
        Drone drone = droneRepository.findByDroneIndex(index)
            .orElseThrow(() -> new RuntimeException("无人机不存在: index=" + index));
        drone.setStatus(status);
        return droneRepository.save(drone);
    }
}
