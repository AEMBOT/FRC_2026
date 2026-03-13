package com.aembot.lib.config.subsystems.intake.generic.run;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXIntakeRollerConfiguration {

  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public double kIntakeVoltage;

  public TalonFXIntakeRollerConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXIntakeRollerConfiguration withIntakeVoltage(double targetSpeed) {
    this.kIntakeVoltage = targetSpeed;
    return this;
  }

  public TalonFXIntakeRollerConfiguration withRealMotorConfiguration(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXIntakeRollerConfiguration withSimMotorConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }
}
