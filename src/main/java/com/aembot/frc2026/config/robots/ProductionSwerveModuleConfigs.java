package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.robots.swerve_tunings.NautilusSwerveConstants; // TODO Need constants from 26 bot
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.config.subsystems.drive.TalonFXSwerveModuleConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.List;

public class ProductionSwerveModuleConfigs {
  public static final int DRIVE_MOTORS_PER_MODULE = 1;
  public static final int STEER_MOTORS_PER_MODULE = 1;

  public final SwerveModuleConfiguration<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeftModule;

  public final SwerveModuleConfiguration<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRightModule;

  public final SwerveModuleConfiguration<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeftModule;

  public final SwerveModuleConfiguration<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRightModule;

  /** List of configurations in FL, FR, BL, BR order */
  public final List<
          SwerveModuleConfiguration<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      configurations;

  public ProductionSwerveModuleConfigs(String driveSubsystemName, String driveCANBusName) {

    this.frontLeftModule =
        new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstants.FrontLeft,
            "FrontLeftSwerveModule",

            // Drive Motor
            new CANDeviceID(
                5,
                "FrontRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                8,
                "FrontLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                26,
                "FrontLeftSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.frontRightModule =
        new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstants.FrontRight,
            "FrontRightSwerveModule",

            // Drive Motor
            new CANDeviceID(
                5,
                "FrontRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                6,
                "FrontRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                24,
                "FrontRightSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.backLeftModule =
        new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstants.BackLeft,
            "BackLeftSwerveModule",

            // Drive Motor
            new CANDeviceID(
                3,
                "BackLeftSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                4,
                "BackLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                25,
                "BackLeftSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.backRightModule =
        new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstants.BackRight,
            "BackRightSwerveModule",

            // Drive Motor
            new CANDeviceID(
                9,
                "BackRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                2,
                "BackRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                23,
                "BackRightSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.configurations =
        List.of(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  }
}
