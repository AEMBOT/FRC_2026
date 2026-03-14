package com.aembot.lib.config.motors;

import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import java.util.ArrayList;
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
public class MotorFollowersConfiguration<C> {
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

    public SimulatedMotorConfiguration<C> simConfig = null;

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
     * Sets the configuration for this follower motor.
     *
     * @param config The {@link SimulatedMotorConfiguration} object containing the follower's
     *     settings.
     * @return This {@link FollowerConfiguration} instance, for chaining.
     */
    public FollowerConfiguration<C> withSimConfig(SimulatedMotorConfiguration<C> config) {
      this.simConfig = config;
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

    /**
     * Check that all values required for a follower motor are set on this config. If they are not,
     * throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
     *
     * @return this {@link FollowerConfiguration} for chaining
     */
    public FollowerConfiguration<C> validate() {
      List<String> missing = new ArrayList<>();
      if (this.config == null) missing.add("config");
      if (this.simConfig == null) missing.add("simConfig");

      if (missing.size() != 0) {
        throw new VerifyError(
            "Config for this follower motor does not have a set " + String.join(",", missing));
      }

      return this;
    }
  }

  public MotorConfiguration<C> leaderConfig;
  public SimulatedMotorConfiguration<C> leaderSimConfig;

  public List<FollowerConfiguration<C>> followerConfigurations = List.of();

  public MotorFollowersConfiguration() {}

  public MotorFollowersConfiguration(
      MotorConfiguration<C> leaderConfig, SimulatedMotorConfiguration<C> leaderSimConfig) {
    this();
    this.leaderConfig = leaderConfig;
    this.leaderSimConfig = leaderSimConfig;
  }

  /**
   * Sets the leader motor configuration for the leader servo motor.
   *
   * @param config The {@link MotorConfiguration} object containing the leader's settings.
   * @return This {@link MotorFollowerConfiguration} instance, for chaining.
   */
  public MotorFollowersConfiguration<C> withLeaderConfig(MotorConfiguration<C> config) {
    this.leaderConfig = config;
    return this;
  }

  /**
   * Sets the leader motor configuration for the leader servo motor.
   *
   * @param config The {@link SimulatedMotorConfiguration} object containing the leader's settings.
   * @return This {@link MotorFollowerConfiguration} instance, for chaining.
   */
  public MotorFollowersConfiguration<C> withLeaderSimConfig(SimulatedMotorConfiguration<C> config) {
    this.leaderSimConfig = config;
    return this;
  }

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

  /**
   * Check that all values required for a motor follower subsystem are set on this config. If they
   * are not, throw a {@link VerifyError}. Intended to be called at the end of an initialization
   * chain.
   *
   * @return this {@link MotorFollowersConfiguration} for chaining
   */
  public MotorFollowersConfiguration<C> validate() {
    List<String> missing = new ArrayList<>();
    if (this.leaderConfig == null) missing.add("leaderConfig");
    if (this.leaderSimConfig == null) missing.add("leaderSimConfig");

    String followerErrors = "";
    for (FollowerConfiguration<C> follower : followerConfigurations) {
      try {
        follower.validate();
      } catch (VerifyError e) {
        followerErrors += e.getMessage() + "\n";
      }
    }

    if (missing.size() != 0 || !followerErrors.isEmpty()) {
      throw new VerifyError(
          "Config for " + leaderConfig != null
              ? leaderConfig.kConfigurationName
              : "UNNAMED"
                  + " does not have a set "
                  + String.join(",", missing)
                  + "\n"
                  + followerErrors);
    }

    return this;
  }
}
