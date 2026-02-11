package com.aembot.frc2026.constants;

import com.aembot.frc2026.config.PlaceholderBot;
import com.aembot.frc2026.config.RobotIDYearly;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public abstract class RobotConfig {
  public abstract String getRobotName();

  public abstract MotorFollowersConfiguration<TalonFXConfiguration> getFlywheelConfiguration();

  public abstract SimulatedFlywheelConfiguration getSimulatedFlywheelConfiguration();

  public static RobotConfig getRobotConstants(RobotIDYearly robotIdentity) {
    switch (robotIdentity) {
      case PRODUCTION:
        return new PlaceholderBot();
      default:
        return new PlaceholderBot();
    }
  }
}
