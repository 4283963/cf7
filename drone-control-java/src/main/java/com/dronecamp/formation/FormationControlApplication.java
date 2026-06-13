package com.dronecamp.formation;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class FormationControlApplication {

    public static void main(String[] args) {
        SpringApplication.run(FormationControlApplication.class, args);
    }
}
