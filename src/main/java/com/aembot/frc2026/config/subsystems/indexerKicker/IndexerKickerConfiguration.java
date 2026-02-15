package com.aembot.frc2026.config.subsystems.indexerKicker;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

/**
 * Config for the kicker subsystem, which moves game pieces from the selector to the shooter
 *
 * <p>Required calls:
 *
 * <ul>
 *   <li>{@link #withMotorConfig(MotorConfiguration)}
 *   <li>{@link #withSimMotorConfig(SimulatedMotorConfiguration)}
 *   <li>{@link #withTargetSpeedRPM(double)}
 *   <li>{@link #validate()} (last)
 * </ul>
 */
public class IndexerKickerConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public Double kTargetSpeedRPM;

  public IndexerKickerConfiguration(String name) {
    this.kName = name;
  }

  /**
   * Set the config for the spindexer's motor
   *
   * @return This {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withMotorConfig(
      MotorConfiguration<TalonFXConfiguration> config) {
    this.kMotorConfig = config;
    return this;
  }

  /**
   * Set the sim config for the spindexer's motor
   *
   * @return This {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> config) {
    this.kSimMotorConfig = config;
    return this;
  }

  /**
   * Set the target RPM of the kicker while running
   *
   * @param targetSpeedRPM target RPM of the kicker while running
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withTargetSpeedRPM(double targetSpeedRPM) {
    this.kTargetSpeedRPM = targetSpeedRPM;
    return this;
  }

  /**
   * Check that all values required for a kicker subsystem are set on this config. If they are not,
   * throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration validate() {
    List<String> missing = new ArrayList<>();
    if (this.kMotorConfig == null) missing.add("kMotorConfig");
    if (this.kSimMotorConfig == null) missing.add("kSimMotorConfig");
    if (this.kTargetSpeedRPM == null) missing.add("kTargetSpeed");

    if (missing.size() != 0) {
      throw new VerifyError(
          "Config for " + kName + " does not have a set " + String.join(",", missing));
    }

    return this;
  }
}
