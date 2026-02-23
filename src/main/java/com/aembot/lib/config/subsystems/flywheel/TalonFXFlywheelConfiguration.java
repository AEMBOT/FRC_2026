package com.aembot.lib.config.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXFlywheelConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** The amount of units we can be off and still shoot */
  public double kAutoAimLeniance;

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

  /**
   * Set the amount of units that we can be off in order to still shoot
   *
   * <p>Counts both directions, so for example if this was 10, we could have a deviance of -10
   * through +10
   *
   * @return this {@link TalonFXFlywheelConfiguration} for chaining
   */
  public TalonFXFlywheelConfiguration withAutoAimLeniance(double autoAimLeniance) {
    this.kAutoAimLeniance = autoAimLeniance;
    return this;
  }
}
