package com.aembot.frc2026.config.robots;

// TODO Need constants from 26 bot
import com.aembot.frc2026.config.robots.swerve_tunings.TimothyTunerConstants;
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
            TimothyTunerConstants.FrontLeft,
            "FrontLeftSwerveModule",

            // Drive Motor
            new CANDeviceID(
                TimothyTunerConstants.FrontLeft.DriveMotorId,
                "FrontLeftSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                TimothyTunerConstants.FrontLeft.SteerMotorId,
                "FrontLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                TimothyTunerConstants.FrontLeft.EncoderId,
                "FrontLeftSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.frontRightModule =
        new TalonFXSwerveModuleConfiguration(
            TimothyTunerConstants.FrontRight,
            "FrontRightSwerveModule",

            // Drive Motor
            new CANDeviceID(
                TimothyTunerConstants.FrontRight.DriveMotorId,
                "FrontRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                TimothyTunerConstants.FrontRight.SteerMotorId,
                "FrontRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                TimothyTunerConstants.FrontRight.EncoderId,
                "FrontRightSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.backLeftModule =
        new TalonFXSwerveModuleConfiguration(
            TimothyTunerConstants.BackLeft,
            "BackLeftSwerveModule",

            // Drive Motor
            new CANDeviceID(
                TimothyTunerConstants.BackLeft.DriveMotorId,
                "BackLeftSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                TimothyTunerConstants.BackLeft.SteerMotorId,
                "BackLeftSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                TimothyTunerConstants.BackLeft.EncoderId,
                "BackLeftSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.backRightModule =
        new TalonFXSwerveModuleConfiguration(
            TimothyTunerConstants.BackRight,
            "BackRightSwerveModule",

            // Drive Motor
            new CANDeviceID(
                TimothyTunerConstants.BackRight.DriveMotorId,
                "BackRightSwerveDriveMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Motor
            new CANDeviceID(
                TimothyTunerConstants.BackRight.SteerMotorId,
                "BackRightSwerveSteerMotor",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX,
                driveCANBusName),

            // Steer Encoder
            new CANDeviceID(
                TimothyTunerConstants.BackRight.EncoderId,
                "BackRightSwerveSteerEncoder",
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER,
                driveCANBusName));

    this.configurations =
        List.of(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  }
}
