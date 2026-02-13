package com.aembot.lib.config.subsystems.intake.overBumper.run;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXOverBumperIntakeRollerConfiguration {

  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public double kTargetSpeedUnitsPerMin;

  public TalonFXOverBumperIntakeRollerConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXOverBumperIntakeRollerConfiguration withTargetSpeed(double targetSpeed) {
    this.kTargetSpeedUnitsPerMin = targetSpeed;
    return this;
  }

  public TalonFXOverBumperIntakeRollerConfiguration withRealMotorConfiguration(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXOverBumperIntakeRollerConfiguration withSimMotorConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }
}
