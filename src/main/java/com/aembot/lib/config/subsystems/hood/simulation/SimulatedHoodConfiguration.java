package com.aembot.lib.config.subsystems.hood.simulation;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulatedHoodConfiguration extends TalonFXHoodConfiguration {

  public SimulatedHoodConfiguration(
      MotorConfiguration<TalonFXConfiguration> motorConfig, String name) {
    super(motorConfig, name);
  }
}
