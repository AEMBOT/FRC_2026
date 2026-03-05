package com.aembot.lib.config.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class TalonFXHoodConfiguration {

  public final String kName;

  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  /** The origin pose of the hood for visualization in advantagescope. */
  public Pose3d kHoodOriginPose;

  /**
   * The robot-relative or turret-relative (if turret present) point at which game-pieces exit the
   * shooter. Used in sim.
   */
  public Translation3d kGamePieceExitPoint = new Translation3d();

  public double upwardsHardStopUnits = 0;

  /** The amount of units we can be off and still shoot */
  public double kAutoAimLeniance;

  public TalonFXHoodConfiguration(
      MotorConfiguration<TalonFXConfiguration> motorConfig, String name) {
    this.kMotorConfig = motorConfig;
    this.kName = name;
  }

  /**
   * Set the origin pose of the hood for visualization in advantagescope.
   *
   * @return This {@link TalonFXHoodConfiguration} for chaining
   */
  public TalonFXHoodConfiguration withHoodOriginPose(Pose3d hoodOriginPose) {
    this.kHoodOriginPose = hoodOriginPose;
    return this;
  }

  /**
   * Set the robot-relative or turret-relative (if turret present) point at which game-pieces exit
   * the shooter. Used in sim.
   *
   * @return This {@link TalonFXHoodConfiguration} for chaining
   */
  public TalonFXHoodConfiguration withGamePieceExitPoint(Translation3d gamePieceExitPoint) {
    this.kGamePieceExitPoint = gamePieceExitPoint;
    return this;
  }

  /**
   * Set the position of the upwards hard stop in units. Used for zeroing.
   *
   * @return This {@link TalonFXHoodConfiguration} for chaining
   */
  public TalonFXHoodConfiguration withUpwardsHardStopUnits(double units) {
    this.upwardsHardStopUnits = units;
    return this;
  }

  /* Set the amount of units that we can be off in order to still shoot
   *
   * <p>Counts both directions, so for example if this was 10, we could have a deviance of -10
   * through +10
   *
   * @return this {@link TalonFXHoodConfiguration} for chaining
   */
  public TalonFXHoodConfiguration withAutoAimLeniance(double autoAimLeniance) {
    this.kAutoAimLeniance = autoAimLeniance;
    return this;
  }
}
