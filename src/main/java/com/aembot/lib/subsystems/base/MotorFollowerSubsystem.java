package com.aembot.lib.subsystems.base;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIO;
import java.util.Arrays;
import java.util.List;
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
public class MotorFollowerSubsystem<I extends MotorInputs, M extends MotorIO, C>
    extends MotorSubsystem<I, M, MotorConfiguration<C>> {
  /** Configuration object defining leader and follower setup. */
  protected MotorFollowersConfiguration<C> mainConfig;

  /** Input/state containers for each follower motor. */
  protected I[] followerMotorInputs;

  /** IO objects for each follower motor. */
  protected List<M> followerMotorIOs;

  /** Container of all motors. Mostly exists to make sure sim notifiers are set up. */
  protected final CompoundMotorIO<M> motorIOContainer;

  public MotorFollowerSubsystem(
      I[] motorInputs, CompoundMotorIO<M> io, MotorFollowersConfiguration<C> config) {
    super(motorInputs[0], io.getMotor(0), config.validate().leaderConfig);

    this.mainConfig = config;
    this.motorIOContainer = io;

    this.followerMotorInputs = Arrays.copyOfRange(motorInputs, 1, motorInputs.length);

    this.followerMotorIOs = io.kMotors.subList(1, io.kMotors.size());

    // Ensure all follower motor property collections are the same length
    assert (this.followerMotorInputs.length == this.followerMotorIOs.size())
            && (this.followerMotorInputs.length == config.followerConfigurations.size())
        : "Length of follower inputs/io/configs not equal";

    // Configure each follower to follow the leader according to their configuration
    for (int i = 0; i < config.followerConfigurations.size(); i++) {
      M motor = this.followerMotorIOs.get(i);
      motor.follow(
          mainConfig.leaderConfig.kCANDevice, config.followerConfigurations.get(i).followDirection);
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    for (int i = 0; i < mainConfig.followerConfigurations.size(); i++) {
      MotorIO followerIO = followerMotorIOs.get(i);
      followerIO.updateInputs(followerMotorInputs[i]);
      Logger.processInputs(
          logPrefixInput
              + "/Followers/"
              + mainConfig.followerConfigurations.get(i).config.kConfigurationName,
          followerMotorInputs[i]);

      followerIO.follow(
          mainConfig.leaderConfig.kCANDevice,
          mainConfig.followerConfigurations.get(i).followDirection);
    }

    // Configure each follower to follow the leader according to their configuration
    // for (int i = 0; i < mainConfig.followerConfigurations.size(); i++) {
    //   M motor = this.followerMotorIOs.get(i);
    //   motor.follow(
    //       mainConfig.leaderConfig.kCANDevice,
    //       mainConfig.followerConfigurations.get(i).followDirection);
    // }
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
   * Get the position of the lead motor
   *
   * @return The position of the lead motor
   */
  @Override
  public double getCurrentPosition() {
    // double uninvertedPositionSum = inputs.positionUnits;
    // for (int i = 0; i < followerMotorInputs.length; i++) {
    //   double followerPosition = followerMotorInputs[i].positionUnits;
    //   // If follower is inverted, negate its position to align with leader
    //   if (mainConfig.followerConfigurations.get(i).followDirection == FollowDirection.INVERT) {
    //     followerPosition = -followerPosition;
    //   }
    //   uninvertedPositionSum += followerPosition;
    // }
    // return uninvertedPositionSum / (mainConfig.followerConfigurations.size() + 1);
    return inputs.positionUnits;
  }

  /**
   * Get the current velocity of the lead motor
   *
   * @return The velocity in user-defined units per second.
   */
  @Override
  public double getCurrentVelocity() {
    // double uninvertedVelocitySum = inputs.velocityUnitsPerSecond;
    // for (int i = 0; i < followerMotorInputs.length; i++) {
    //   double followerVelocity = followerMotorInputs[i].velocityUnitsPerSecond;
    //   // If follower is inverted, negate its velocity to align with leader
    //   if (mainConfig.followerConfigurations.get(i).followDirection == FollowDirection.INVERT) {
    //     followerVelocity = -followerVelocity;
    //   }
    //   uninvertedVelocitySum += followerVelocity;
    // }
    // return uninvertedVelocitySum / (mainConfig.followerConfigurations.size() + 1);
    return inputs.velocityUnitsPerSecond;
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
