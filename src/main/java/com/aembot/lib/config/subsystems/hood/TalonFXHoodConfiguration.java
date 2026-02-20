package com.aembot.lib.config.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;

public class TalonFXHoodConfiguration {

  public final String kName;

  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  /** The origin pose of the hood for visualization in advantagescope. */
  public Pose3d kHoodOriginPose;

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
}
