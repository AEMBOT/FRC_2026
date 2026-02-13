package com.aembot.lib.config.subsystems.intake.over_bumper.run.simulation;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedOverBumperIntakeRollerConfiguration
    extends TalonFXOverBumperIntakeRollerConfiguration {

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedOverBumperIntakeRollerConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    super(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }
}
