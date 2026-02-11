package com.aembot.lib.config.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXHoodConfiguration {

  public final String kName;

  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public TalonFXHoodConfiguration(
      MotorConfiguration<TalonFXConfiguration> motorConfig, String name) {
    this.kMotorConfig = motorConfig;
    this.kName = name;
  }
}
