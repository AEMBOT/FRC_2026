package com.aembot.frc2026.config;

import com.aembot.frc2026.config.robots.ProductionConfig;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.config.RobotConfig;
import java.util.List;

/**
 * Abstract class defining how a robot should be configured. Contains method to get the {@link
 * RobotConfig} implementation for a given {@link RobotIDYearly}
 *
 * @see RobotConfiguration#getRobotConstants(RobotIDYearly)
 */
public abstract class RobotConfiguration {
  /**
   * Get the name of the robot has a human readable string
   *
   * @return Robot name as a string
   */
  public abstract String getRobotName();

  /**
   * Gets the list of named CAN buses that are present on this bot
   *
   * @return Current list of CAN buses used on this robot
   */
  public abstract List<String> getCANBusNames();

  /**
   * Configuration that will be passed into the drivetrain configuration
   *
   * @return The current robot drivetrain configuration
   */
  public abstract DrivetrainConfiguration getDrivetrainConfiguration();

  /**
   * Retrieve the configuration for the swerve modules on the robot
   *
   * @return List of SwerveModuleConfigurations in FL, FR, BL, BR order
   */
  public abstract List<
          SwerveModuleConfiguration<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      getSwerveConfigurations();

  /**
   * Get configuration details about the drive train sim for this robot
   *
   * @return DrivetrainSimConfiguration of the given robot
   */
  public abstract DrivetrainSimConfiguration getSimulatedDrivetrainConfiguration();

  /**
   * retrieve the correct robot constants based on the given robot identification
   *
   * @param identification Robot identification representing what robot is currently in use
   * @return The desired configuration that should be utilized with this robot
   */
  public static RobotConfiguration getRobotConstants(RobotIDYearly identification) {
    switch (identification) {
      case PRODUCTION:
      default:
        return new ProductionConfig();
    }
  }
}
