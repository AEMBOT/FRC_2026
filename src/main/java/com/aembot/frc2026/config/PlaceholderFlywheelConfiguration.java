package com.aembot.frc2026.config;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.motors.factories.TalonFXFactoryConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.aembot.lib.config.wrappers.ConfigureSlot0Gains;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.List;

public class PlaceholderFlywheelConfiguration {
  public final double MAX_VELOCITY = 100.0;
  public final double MIN_VELOCITY = 3.0;
  public final double MAX_ACCELERATION = 3.0;
  public final double JERK = 0.0;

  public final ConfigureSlot0Gains MOTOR_GAINS =
      new ConfigureSlot0Gains(3.0, 0.0, 0.0, 0.3, 0.1, 0.0, 0.0);

  public final SimulatedFlywheelConfiguration FLYWHEEL_SIM_CONFIG;
  public final MotorFollowersConfiguration<TalonFXConfiguration> FLYWHEEL_SUBSYSTEM_CONFIGURATION;

  private static final int LEADER_MOTOR_ID = 12;
  private static final String LEADER_MOTOR_NAME = "FlywheelMotor";
  private static final double LEADER_MOTOR_CURRENT_LIMIT = 50.0;

  private static final int FOLLOWER_MOTOR_ID = 13;
  private static final String FOLLOWER_MOTOR_NAME = "FlywheelMotorFollower";
  private static final double FOLLOWER_MOTOR_CURRENT_LIMIT = 50.0;

  private static final String FLYWHEEL_SUBSYTEM_NAME = "FlywheelSubsystem";

  public PlaceholderFlywheelConfiguration(String CANBusName) {
    // sim configuration
    // placeholder values
    FLYWHEEL_SIM_CONFIG = new SimulatedFlywheelConfiguration();
    FLYWHEEL_SIM_CONFIG.JKgMetersSquared = 4;
    FLYWHEEL_SIM_CONFIG.gearing = 2;
    FLYWHEEL_SIM_CONFIG.radToRotorRatio = 3;

    // followers configuration
    MotorFollowersConfiguration.FollowerConfiguration<TalonFXConfiguration> followerConfig =
        TalonFXFactoryConfiguration.generateFollowerTalonFXConfiguration();
    followerConfig.config.configurationName = "FlywheelMotorFollower";
    followerConfig.config.canDevice =
        new CANDeviceID(
            FOLLOWER_MOTOR_ID,
            FOLLOWER_MOTOR_NAME,
            FLYWHEEL_SUBSYTEM_NAME,
            CANDeviceID.CANDeviceType.TALON_FX,
            CANBusName);

    followerConfig.config.motorConfig.CurrentLimits.StatorCurrentLimit =
        FOLLOWER_MOTOR_CURRENT_LIMIT;
    followerConfig.config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    followerConfig.followDirection = FollowDirection.SAME;
    followerConfig.config.motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // subsystem configuration
    FLYWHEEL_SUBSYSTEM_CONFIGURATION =
        new MotorFollowersConfiguration<>(new TalonFXConfiguration());

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.configurationName = "FlywheelSubsystem";
    FLYWHEEL_SUBSYSTEM_CONFIGURATION.canDevice =
        new CANDeviceID(
            LEADER_MOTOR_ID,
            LEADER_MOTOR_NAME,
            FLYWHEEL_SUBSYTEM_NAME,
            CANDeviceID.CANDeviceType.TALON_FX,
            CANBusName);

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.Slot0 = MOTOR_GAINS;

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        MAX_VELOCITY;
    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.MotionMagic.MotionMagicAcceleration =
        MAX_ACCELERATION;
    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.MotionMagic.MotionMagicJerk = JERK;

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.CurrentLimits.StatorCurrentLimit =
        LEADER_MOTOR_CURRENT_LIMIT;
    FLYWHEEL_SUBSYSTEM_CONFIGURATION.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    FLYWHEEL_SUBSYSTEM_CONFIGURATION.followerConfigurations = List.of(followerConfig);
  }
}
