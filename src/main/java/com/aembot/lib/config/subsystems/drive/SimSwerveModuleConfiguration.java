package com.aembot.lib.config.subsystems.drive;

import com.aembot.lib.core.motors.io.MotorIOTalonFXCANCoderSim;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Handles basic swerve module configuration wrapper for simulated swerve modules when using
 * MapleSim drivetrain
 */
public class SimSwerveModuleConfiguration {
  public final SwerveModuleConstants<?, ?, ?> moduleConstants;
  public final SwerveModuleSimulation
      mapleSimModule; // Maple sim swerve module simulation component

  public SimSwerveModuleConfiguration(
      SwerveModuleConstants<?, ?, ?> moduleConstants,
      SwerveModuleSimulation moduleSimulation,
      SwerveModule<TalonFX, TalonFX, CANcoder> module,
      SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          moduleConfig) {
    this.moduleConstants = moduleConstants;

    // Setup module simulation
    this.mapleSimModule = moduleSimulation;

    // Configure drive motor controller to use a talon FX Sim IO
    this.mapleSimModule.useDriveMotorController(new MotorIOTalonFXSim(module.getDriveMotor()));

    // Configure steer motor controller to use a Talon FX wth Sim CAN coder
    this.mapleSimModule.useSteerMotorController(
        new MotorIOTalonFXCANCoderSim(module.getSteerMotor(), module.getEncoder()));
  }
}
