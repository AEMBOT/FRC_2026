package com.aembot.frc2026.config.subsystems.indexerSelector;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

/**
 * Config for the selector subsystem, which moves game pieces from the spindexer to the kicker
 *
 * <p>Required calls:
 *
 * <ul>
 *   <li>{@link #withMotorConfig(MotorConfiguration)}
 *   <li>{@link #withTimeOfFlightConfig(TimeOfFlightConfiguration)}
 *   <li>{@link #withTargetSpeedRPM(double)}
 *   <li>{@link #validate()} (last)
 * </ul>
 */
public class IndexerSelectorConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;
  public TimeOfFlightConfiguration kTimeOfFlightConfig;

  public Double kTargetSpeedRPM;

  /**
   * The amount of time it takes to transport a game piece from the this indexer stage to the next.
   * Used for timeouts and sim.
   */
  public Double kGamePieceMoveTime;

  /** The number of game pieces this indexer stage can hold. Used for sim. */
  public Integer kGamePieceCapacity;

  public IndexerSelectorConfiguration(String name) {
    this.kName = name;
  }

  /**
   * Set the config for the selector's motor
   *
   * @return This {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withMotorConfig(
      MotorConfiguration<TalonFXConfiguration> config) {
    this.kMotorConfig = config;
    return this;
  }

  /**
   * Set the sim config for the spindexer's motor
   *
   * @return This {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> config) {
    this.kSimMotorConfig = config;
    return this;
  }

  /**
   * Set the config for the selector's time of flight sensor
   *
   * @return This {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withTimeOfFlightConfig(TimeOfFlightConfiguration config) {
    this.kTimeOfFlightConfig = config;
    return this;
  }

  /**
   * Set the target RPM of the selector while running
   *
   * @param targetSpeedRPM target RPM of the selector while running
   * @return this {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withTargetSpeedRPM(double targetSpeedRPM) {
    this.kTargetSpeedRPM = targetSpeedRPM;
    return this;
  }

  /**
   * Set the time it takes in seconds for a game piece to travel through this indexer stage to the
   * next
   *
   * @return this {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withGamePieceMoveTimeSeconds(double seconds) {
    this.kGamePieceMoveTime = seconds;
    return this;
  }

  /**
   * Set the number of game pieces this indexer stage is able to hold. Used in sim.
   *
   * @return this {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration withGamePieceCapacity(int numGamePieces) {
    this.kGamePieceCapacity = numGamePieces;
    return this;
  }

  /**
   * Check that all values required for a selector subsystem are set on this config. If they are
   * not, throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link IndexerSelectorConfiguration} for chaining
   */
  public IndexerSelectorConfiguration validate() {
    List<String> missing = new ArrayList<>();
    if (this.kTargetSpeedRPM == null) missing.add("kTargetSpeed");
    if (this.kGamePieceMoveTime == null) missing.add("kGamePieceMoveTime");
    if (this.kGamePieceCapacity == null) missing.add("kGamePieceCapacity");
    if (this.kMotorConfig == null) missing.add("kMotorConfig");
    if (this.kSimMotorConfig == null) missing.add("kSimMotorConfig");
    if (this.kTimeOfFlightConfig == null) missing.add("kTimeOfFlightConfig");

    if (missing.size() != 0) {
      throw new VerifyError(
          "Config for " + kName + " does not have a set " + String.join(",", missing));
    }

    return this;
  }
}
