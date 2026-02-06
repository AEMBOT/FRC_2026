package com.aembot.lib.config.subsystems.hood.simulation;

import com.aembot.lib.config.motors.factories.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedHoodConfiguration extends TalonFXHoodConfiguration {

  public final SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public SimulatedHoodConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig, String name) {
    super(simMotorConfig.kRealConfiguration, name);
    this.kSimMotorConfig = simMotorConfig;
  }
}
