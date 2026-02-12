package com.aembot.lib.config.subsystems.intake.over_bumper.deploy;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXOverBumperIntakeDeployConfiguration {

  public final String kName;

  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public TalonFXOverBumperIntakeDeployConfiguration(
      MotorConfiguration<TalonFXConfiguration> motorConfig, String name) {
    this.kMotorConfig = motorConfig;
    this.kName = name;
  }
}
