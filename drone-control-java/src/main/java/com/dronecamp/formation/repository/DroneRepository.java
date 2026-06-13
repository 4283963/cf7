package com.dronecamp.formation.repository;

import com.dronecamp.formation.entity.Drone;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import java.util.Optional;

@Repository
public interface DroneRepository extends JpaRepository<Drone, Long> {
    Optional<Drone> findByDroneCode(String droneCode);
    Optional<Drone> findByDroneIndex(Integer droneIndex);
}
