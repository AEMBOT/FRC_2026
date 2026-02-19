package com.aembot.lib.config.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXFlywheelConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public TalonFXFlywheelConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXFlywheelConfiguration withRealMotorConfig(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXFlywheelConfiguration withSimulatedMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }
}
