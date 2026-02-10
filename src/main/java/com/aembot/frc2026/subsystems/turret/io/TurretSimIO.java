// also copy pasted modified from elevatorsimio.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package com.aembot.frc2026.subsystems.turret.io;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.aembot.lib.config.motors.MotorFollowersConfiguration;
// import com.aembot.lib.config.subsystems.turret.simulation.SimulatedTurretConfiguration;
// import com.aembot.lib.core.motors.interfaces.MotorIO;
// import com.aembot.lib.subsystems.turret.simulation.SimulatedElevator; // add this next

// /** Elevator IO implementation for simulation */
// public class TurretSimIO extends ElevatorHardwareIO {
//     private final SimulatedElevator simElevator;

//     public TurretSimIO(
//         MotorFollowersConfiguration<TalonFXConfiguration> elevatorConfig,
//         SimulatedElevatorConfiguration simulatedElevatorConfig
//     ){
//         super(elevatorConfig);
//         simElevator = new SimulatedElevator(elevatorConfig, simulatedElevatorConfig);
//     }

//     @Override
//     public void updateLog(String standardPrefix, String inputPrefix) {
//         simElevator.updateLog(standardPrefix, inputPrefix);
//     }

//     @Override
//     public MotorIO getLeadMotor() {
//         return simElevator.getLeadTalon();
//     }

//     @Override
//     public MotorIO[] getFollowerMotors() {
//         return simElevator.getFollowerTalons();
//     }

// }

// package com.aembot.frc2026.subsystems.turret.io;

// public class TurretSimIO {

// }
