package com.aembot.frc2026.config.subsystems.indexerKicker;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Transform3d;
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
 *   <li>{@link #withResistSpeedRPM(double)}
 *   <li>{@link #validate()} (last)
 * </ul>
 */
public class IndexerKickerConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public Double kTargetSpeedRPM;

  /** Speed to run the kicker at to prevent game pieces from being pushed into the shooter. */
  public Double kResistSpeedRPM;

  /**
   * The amount of time it takes to transport a game piece from the this indexer stage to the next.
   * Used in sim.
   */
  public Double kGamePieceMoveTime;

  /** The number of game pieces this indexer stage can hold. Used for sim. */
  public Integer kGamePieceCapacity;

  /** Poses that game pieces can appear in. Used for visualization in sim. */
  public Transform3d[] kGamePiecePositions = {new Transform3d()};

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
   * Set the speed to run the kicker at to prevent game pieces from being pushed into the shooter
   *
   * @param resistSpeedRPM target RPM of the kicker while resisting
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withResistSpeedRPM(double resistSpeedRPM) {
    this.kResistSpeedRPM = resistSpeedRPM;
    return this;
  }

  /**
   * Set the time it takes in seconds for a game piece to travel through this indexer stage to the
   * next
   *
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withGamePieceMoveTimeSeconds(double seconds) {
    this.kGamePieceMoveTime = seconds;
    return this;
  }

  /**
   * Set the number of game pieces this indexer stage is able to hold. Used in sim.
   *
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withGamePieceCapacity(int numGamePieces) {
    this.kGamePieceCapacity = numGamePieces;
    return this;
  }

  /**
   * Set poses that game pieces can appear in. Used for visualization in sim.
   *
   * @return this {@link IndexerKickerConfiguration} for chaining
   */
  public IndexerKickerConfiguration withGamePiecePositions(Transform3d[] positions) {
    this.kGamePiecePositions = positions;
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
    if (this.kResistSpeedRPM == null) missing.add("kResistSpeedRPM");
    if (this.kGamePieceMoveTime == null) missing.add("kGamePieceMoveTime");
    if (this.kGamePieceCapacity == null) missing.add("kGamePieceCapacity");

    if (missing.size() != 0) {
      throw new VerifyError(
          "Config for " + kName + " does not have a set " + String.join(",", missing));
    }

    return this;
  }
}
