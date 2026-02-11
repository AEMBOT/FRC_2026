package com.aembot.frc2026.config;

import com.aembot.frc2026.constants.RobotConfig;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class PlaceholderBot extends RobotConfig {
  private static final String ROBOT_NAME = "Placeholder";

  private static final String CANBUS_NAME = "CANivore";

  private static final PlaceholderFlywheelConfiguration FLYWHEEL_CONFIGURATION =
      new PlaceholderFlywheelConfiguration(CANBUS_NAME);

  @Override
  public String getRobotName() {
    return ROBOT_NAME;
  }

  @Override
  public SimulatedFlywheelConfiguration getSimulatedFlywheelConfiguration() {
    return FLYWHEEL_CONFIGURATION.FLYWHEEL_SIM_CONFIG;
  }

  @Override
  public MotorFollowersConfiguration<TalonFXConfiguration> getFlywheelConfiguration() {
    return FLYWHEEL_CONFIGURATION.FLYWHEEL_SUBSYSTEM_CONFIGURATION;
  }
}
