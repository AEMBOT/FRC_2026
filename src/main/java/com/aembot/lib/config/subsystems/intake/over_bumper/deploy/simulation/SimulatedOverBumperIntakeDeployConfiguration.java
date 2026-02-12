package com.aembot.lib.config.subsystems.intake.over_bumper.deploy.simulation;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeDeployConfiguration
    extends TalonFXOverBumperIntakeDeployConfiguration {

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeDeployConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    super(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }
}
