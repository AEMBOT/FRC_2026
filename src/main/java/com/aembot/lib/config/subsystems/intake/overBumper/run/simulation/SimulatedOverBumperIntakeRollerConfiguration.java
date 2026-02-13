package com.aembot.lib.config.subsystems.intake.overBumper.run.simulation;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeRollerConfiguration {
  public TalonFXOverBumperIntakeRollerConfiguration kRealConfig;

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeRollerConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    this.kRealConfig =
        new TalonFXOverBumperIntakeRollerConfiguration(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }

  public SimulatedOverBumperIntakeRollerConfiguration withRealConfig(
      TalonFXOverBumperIntakeRollerConfiguration realConfig) {
    this.kRealConfig = realConfig;
    return this;
  }
}
