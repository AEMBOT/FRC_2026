package com.aembot.lib.config.subsystems.intake.overBumper.run;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXOverBumperIntakeRollerConfiguration {

  public final String kName;

  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public double kTargetSpeedUnitsPerMin;

  public TalonFXOverBumperIntakeRollerConfiguration(
      MotorConfiguration<TalonFXConfiguration> motorConfig, String name) {
    this.kMotorConfig = motorConfig;
    this.kName = name;
  }

  public TalonFXOverBumperIntakeRollerConfiguration withTargetSpeed(double targetSpeed) {
    this.kTargetSpeedUnitsPerMin = targetSpeed;
    return this;
  }
}
