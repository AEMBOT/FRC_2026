package com.aembot.frc2026.config.subsystems.spindexer;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

/**
 * Config for the spindexer subsystem, which moves game pieces from the intake to the selector
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
public class SpindexerConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public Double kTargetSpeedRPM;

  /**
   * The amount of time it takes to transport a game piece from the this indexer stage to the next.
   * Used for timeouts and sim.
   */
  public Double kGamePieceMoveTime;

  public SpindexerConfiguration(String name) {
    this.kName = name;
  }

  /**
   * Set the config for the spindexer's motor
   *
   * @return This {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration withMotorConfig(MotorConfiguration<TalonFXConfiguration> config) {
    this.kMotorConfig = config;
    return this;
  }

  /**
   * Set the sim config for the spindexer's motor
   *
   * @return This {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> config) {
    this.kSimMotorConfig = config;
    return this;
  }

  /**
   * Set the target RPM of the spindexer while running
   *
   * @param targetSpeedRPM target RPM of the spindexer while running
   * @return this {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration withTargetSpeedRPM(double targetSpeedRPM) {
    this.kTargetSpeedRPM = targetSpeedRPM;
    return this;
  }

  /**
   * Set the time it takes in seconds for a game piece to travel through this indexer stage to the
   * next
   *
   * @return this {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration withGamePieceMoveTimeSeconds(double seconds) {
    this.kGamePieceMoveTime = seconds;
    return this;
  }

  /**
   * Check that all values required for a spindexer subsystem are set on this config. If they are
   * not, throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration validate() {
    List<String> missing = new ArrayList<>();
    if (this.kTargetSpeedRPM == null) missing.add("kTargetSpeed");
    if (this.kGamePieceMoveTime == null) missing.add("kGamePieceMoveTime");
    if (this.kMotorConfig == null) missing.add("kMotorConfig");
    if (this.kSimMotorConfig == null) missing.add("kSimMotorConfig");

    if (missing.size() != 0) {
      throw new VerifyError(
          "Config for " + kName + " does not have a set " + String.join(",", missing));
    }

    return this;
  }
}
