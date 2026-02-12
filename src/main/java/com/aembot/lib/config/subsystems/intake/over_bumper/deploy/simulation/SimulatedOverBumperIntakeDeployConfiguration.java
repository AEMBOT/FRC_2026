package com.aembot.lib.config.subsystems.intake.over_bumper.deploy.simulation;

import com.aembot.lib.config.motors.SimulatedFlywheelConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeDeployConfiguration
    extends TalonFXOverBumperIntakeDeployConfiguration {

  public final SimulatedFlywheelConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeDeployConfiguration(
      SimulatedFlywheelConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    super(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }
}
