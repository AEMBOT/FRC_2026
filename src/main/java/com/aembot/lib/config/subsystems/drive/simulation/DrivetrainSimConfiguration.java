package com.aembot.lib.config.subsystems.drive.simulation;

import com.aembot.lib.config.robot.PhysicalConfiguration;

public class DrivetrainSimConfiguration {
  /** Name of the config */
  public String kConfigurationName;

  /** Rate in seconds that the sim will loop */
  public final double simLoopPeriodS;

  public PhysicalConfiguration physicalConfiguration;

  /** Number of drive motors per swerve modules */
  public int driveMotorsPerModule;

  /** Number of steer motors per swerve modules */
  public int steerMotorsPerModule;

  public DrivetrainSimConfiguration(double loopPeriod) {
    this.simLoopPeriodS = loopPeriod;
  }

  public DrivetrainSimConfiguration withPhysicalConfiguration(PhysicalConfiguration config) {
    this.physicalConfiguration = config;
    return this;
  }

  public DrivetrainSimConfiguration withDriveMotorsPerModule(int driveMotorCount) {
    this.driveMotorsPerModule = driveMotorCount;
    return this;
  }

  public DrivetrainSimConfiguration withSteerMotorsPerModule(int steerMotorCount) {
    this.steerMotorsPerModule = steerMotorCount;
    return this;
  }

  public DrivetrainSimConfiguration withName(String name) {
    this.kConfigurationName = name;
    return this;
  }
}
