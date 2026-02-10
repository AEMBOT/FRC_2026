package com.aembot.frc2026.config.robots;

// TODO Need constants from 26 bot
import com.aembot.frc2026.config.robots.swerve_tunings.NautilusSwerveConstants;
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
                NautilusSwerveConstants.FrontLeft.DriveMotorId,
                "FrontLeftSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                NautilusSwerveConstants.FrontLeft.SteerMotorId,
                "FrontLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                NautilusSwerveConstants.FrontLeft.EncoderId,
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
                NautilusSwerveConstants.FrontRight.DriveMotorId,
                "FrontRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                NautilusSwerveConstants.FrontRight.SteerMotorId,
                "FrontRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                NautilusSwerveConstants.FrontRight.EncoderId,
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
                NautilusSwerveConstants.BackLeft.DriveMotorId,
                "BackLeftSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                NautilusSwerveConstants.BackLeft.SteerMotorId,
                "BackLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                NautilusSwerveConstants.BackLeft.EncoderId,
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
                NautilusSwerveConstants.BackRight.DriveMotorId,
                "BackRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                NautilusSwerveConstants.BackRight.SteerMotorId,
                "BackRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                NautilusSwerveConstants.BackRight.EncoderId,
                "BackRightSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.configurations =
        List.of(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  }
}
