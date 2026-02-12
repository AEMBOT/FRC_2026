package com.aembot.lib.config.subsystems.intake.over_bumper.run.simulation;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.run.TalonFXOverBumperIntakeRunConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeRunConfiguration
    extends TalonFXOverBumperIntakeRunConfiguration {

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeRunConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    super(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }
}
