package com.aembot.lib.config.subsystems.intake.overBumper.deploy.simulation;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeDeployConfiguration {

  public TalonFXOverBumperIntakeDeployConfiguration kRealConfig;

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeDeployConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    kRealConfig =
        new TalonFXOverBumperIntakeDeployConfiguration(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }

  public SimulatedOverBumperIntakeDeployConfiguration withRealConfig(
      TalonFXOverBumperIntakeDeployConfiguration realConfig) {
    this.kRealConfig = realConfig;
    return this;
  }
}
