package com.aembot.lib.config.motors;

import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import java.util.List;

/**
 * Configuration for a leader motor, with a set of follower motors.
 *
 * <p>Defines how follower motors are set up relative to a leader motor. Each follower motor can
 * have its own {@link MotorConfiguration} and may be set to follow in the same or inverted
 * direction to the leader.
 *
 * @param <C> The type of the underlying configuration object used for each motor. Ex:
 *     TalonFXConfiguration
 */
public class MotorFollowersConfiguration<C> extends MotorConfiguration<C> {
  /**
   * Defines how an individual follower motor is configured relative to a leader motor.
   *
   * <p>Each follower motor includes its own configuration and an inversion flag.
   *
   * @param <C> The type of motor configuration object associated with the follower motor.
   */
  public static class FollowerConfiguration<C> {
    /** The direction in which this follower should follow the leader. */
    public FollowDirection followDirection = FollowDirection.SAME;

    /** Config of the follower motor */
    public MotorConfiguration<C> config = null;

    public FollowerConfiguration(MotorConfiguration<C> config) {
      this.config = config;
    }

    /**
     * Sets the configuration for this follower motor.
     *
     * @param config The {@link MotorConfiguration} object containing the follower's settings.
     * @return This {@link FollowerConfiguration} instance, for chaining.
     */
    public FollowerConfiguration<C> withConfig(MotorConfiguration<C> config) {
      this.config = config;
      return this;
    }

    /**
     * Sets the follow direction for this follower motor.
     *
     * @param direction The {@link FollowDirection} indicating whether the follower should mirror or
     *     invert the leader's movement.
     * @return This {@link FollowerConfiguration} instance, for chaining.
     */
    public FollowerConfiguration<C> withFollowDirection(FollowDirection direction) {
      this.followDirection = direction;
      return this;
    }
  }

  public List<FollowerConfiguration<C>> followerConfigurations = List.of();

  public MotorFollowersConfiguration(C config) {
    super.withConfig(config);
  }

  public MotorFollowersConfiguration() {}

  /**
   * Sets the follower motor configurations for the follower servo motors.
   *
   * @param motors A list of {@link FollowerConfiguration} objects defining each follower motor.
   * @return This {@link MotorFollowerConfiguration} instance, for chaining.
   */
  public MotorFollowersConfiguration<C> withFollowerConfigs(
      List<FollowerConfiguration<C>> configs) {
    this.followerConfigurations = configs;
    return this;
  }
}
