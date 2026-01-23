package com.aembot.lib.subsystems.base;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import org.littletonrobotics.junction.Logger;

/**
 * Flexible base subsystem for any {@link MotorIO}-driven mechanism that can behave like a servo
 * with multiple followers. It wraps a configurable motor controller, handles telemetry, and exposes
 * commands for open-loop and closed-loop position/velocity control.
 *
 * <p>Key capabilities provided by this class include:
 *
 * <ul>
 *   <li>Polling {@link MotorInputs} and pushing telemetry to {@link
 *       org.littletonrobotics.junction.Logger AKit's Logger}
 *   <li>Helper methods for managing encoder offsets and setpoints
 *   <li>Command factories for controlling with Motion Magic, PID, & duty-cycle commands
 * </ul>
 *
 * @param <I> {@link MotorInputs} class capturing feedback for this motor.
 * @param <M> {@link MotorIO} IO interface for the motor.
 * @param <C> {@link MotorFollowerConfiguration} describing behaviour of the motor & its followersx
 */
public class MotorFollowerSubsystem<
        I extends MotorInputs, M extends MotorIO, C extends MotorFollowersConfiguration<?>>
    extends MotorSubsystem<I, M, C> {
  /** Configuration object defining leader and follower setup. */
  protected C mainConfig;

  /** Input/state containers for each follower motor. */
  protected I[] followerMotorInputs;

  /** IO objects for each follower motor. */
  protected M[] followerMotorIOs;

  /**
   * Constructs a new {@code MotorFollowerSubsystem}.
   *
   * @param leaderMotorInputs The inputs object representing the leader motor’s sensor readings.
   * @param leaderMotor The {@link MotorIO} implementation controlling the leader motor.
   * @param followerMotorInputs Array of inputs for each follower motor.
   * @param followerMotors Array of {@link MotorIO} objects for each follower motor.
   * @param config The configuration object defining follower relationships and parameters.
   * @throws AssertionError if the number of follower inputs does not match the number of follower
   *     motors.
   */
  public MotorFollowerSubsystem(
      I leaderMotorInputs, M leaderMotor, I[] followerMotorInputs, M[] followerMotors, C config) {
    super(leaderMotorInputs, leaderMotor, config);

    this.mainConfig = config;

    this.followerMotorInputs = followerMotorInputs;
    this.followerMotorIOs = followerMotors;

    // Ensure all follower motor property collections are the same length
    assert (this.followerMotorInputs.length == this.followerMotorIOs.length)
            && (this.followerMotorInputs.length == config.followerConfigurations.size())
        : "Length of follower inputs/io/configs not equal";

    // Configure each follower to follow the leader according to their configuration
    for (int i = 0; i < config.followerConfigurations.size(); i++) {
      MotorIO motor = followerMotors[i];
      motor.follow(mainConfig.canDevice, config.followerConfigurations.get(i).followDirection);
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    for (int i = 0; i < mainConfig.followerConfigurations.size(); i++) {
      MotorIO followerIO = followerMotorIOs[i];
      followerIO.updateInputs(followerMotorInputs[i]);
      Logger.processInputs(logPrefixInput + "/Inputs", followerMotorInputs[i]);
    }
  }

  /**
   * Sets the encoder position for both the leader and all follower motors.
   *
   * @param position The new encoder position (in configured units).
   */
  @Override
  protected void setEncoderPosition(double position) {
    super.setEncoderPosition(position);
    for (M follower : followerMotorIOs) {
      follower.setCurrentEncoderPosition(position);
    }
  }

  /** Zeros the encoder position for both the leader and all follower motors. */
  @Override
  protected void zeroEncoderPosition() {
    super.zeroEncoderPosition();
    for (M follower : followerMotorIOs) {
      follower.zeroEncoderPosition();
    }
  }

  /**
   * Computes the average current position across the leader and all followers.
   *
   * @return The average position in user-defined units.
   */
  @Override
  public double getCurrentPosition() {
    double uninvertedPositionSum = inputs.positionUnits;
    for (int i = 0; i < followerMotorInputs.length; i++) {
      double followerPosition = followerMotorInputs[i].positionUnits;
      // If follower is inverted, negate its position to align with leader
      if (mainConfig.followerConfigurations.get(i).followDirection == FollowDirection.INVERT) {
        followerPosition = -followerPosition;
      }
      uninvertedPositionSum += followerPosition;
    }
    return uninvertedPositionSum / (mainConfig.followerConfigurations.size() + 1);
  }

  /**
   * Computes the average current velocity across the leader and all followers.
   *
   * @return The average velocity in user-defined units per second.
   */
  @Override
  public double getCurrentVelocity() {
    double uninvertedVelocitySum = inputs.velocityUnitsPerSecond;
    for (int i = 0; i < followerMotorInputs.length; i++) {
      double followerVelocity = followerMotorInputs[i].velocityUnitsPerSecond;
      // If follower is inverted, negate its velocity to align with leader
      if (mainConfig.followerConfigurations.get(i).followDirection == FollowDirection.INVERT) {
        followerVelocity = -followerVelocity;
      }
      uninvertedVelocitySum += followerVelocity;
    }
    return uninvertedVelocitySum / (mainConfig.followerConfigurations.size() + 1);
  }

  /**
   * Helper function to generate a list of {@link MotorInputs} objects for each provided follower
   * motor
   *
   * @param motors List of motors the inputs are being generated for
   * @return Array of motor inputs that were generated to match
   */
  protected static MotorInputs[] generateFollowerInputs(MotorIO[] motors) {
    MotorInputs[] inputs = new MotorInputs[motors.length];
    for (int i = 0; i < motors.length; i++) {
      inputs[i] = new MotorInputs();
    }
    return inputs;
  }
}
