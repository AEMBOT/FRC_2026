package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.robot.PhysicalConfiguration;
import com.aembot.lib.config.robot.Pigeon2GyroConfiguration;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class ProductionDrivetrainConfig {
  public final class GeneralConstants {
    public static final String SUBSYSTEM_NAME = "DriveSubsystem";
  }

  /* ---- GYRO ---- */
  private final class GyroConstants {
    static final Rotation3d MOUNT_ROTATION =
        new Rotation3d(
            Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0));

    static final String NAME = "DrivetrainGyro";

    static final int ID = 1;

    static Pigeon2GyroConfiguration makeGyroConfiguration(String canBusName) {
      return new Pigeon2GyroConfiguration()
          .withGyroMountRotation(MOUNT_ROTATION)
          .withCANDevice(
              new CANDeviceID(
                  ID,
                  NAME,
                  GeneralConstants.SUBSYSTEM_NAME,
                  CANDeviceID.CANDeviceType.PIGEON2,
                  canBusName));
    }
  }

  /* ---- DRIVETRAIN ---- */
  private final class DrivetrainConstants {
    /** Odometry standard deviations while disabled */
    private static final OdometryStandardDevs DISABLED_STANDARD_DEVS =
        new OdometryStandardDevs(1, 1, 1);

    /** Odometry standard deviations while enabled */
    private static final OdometryStandardDevs ENABLED_STANDARD_DEVS =
        new OdometryStandardDevs(0.3, 0.3, 0.2);

    /** Any input less than this chassis speed in meters per second will be set to 0 */
    public static final double CHASSIS_TRANSLATION_SPEED_THRESHOLD = 0.05;

    /** Any input less than this chassis speed in radians per second will be set to 0 */
    public static final double CHASSIS_ROTATION_SPEED_THRESHOLD = 0.05;

    /** Any joystick input less than this on the drive joystick will be ignored */
    public static final double JOYSTICK_TRANSLATION_DEADBAND = 0.05;

    /** Any joystick input less than this on the steer joystick will be ignored */
    public static final double JOYSTICK_STEER_DEADBAND = 0.05;

    /**
     * The max horizontal speed the drivetrain will drive at in m/s. Not necessarily the physical
     * limit.
     */
    public static final double MAX_DRIVE_SPEED = 3.6;

    /**
     * The max angular speed the drivetrain will drive at in rads/s. Not necessarily the physical
     * limit.
     */
    public static final double MAX_ANGULAR_RATE = 8.2;

    private static DrivetrainConfiguration makeDrivetrainConfiguration(
        String canBusName,
        ProductionSwerveModuleConfigs moduleConfigs,
        Pigeon2GyroConfiguration gyroConfiguration) {
      return new DrivetrainConfiguration()
          .withName(GeneralConstants.SUBSYSTEM_NAME)
          .withMaxDriveSpeed(MAX_DRIVE_SPEED)
          .withMaxAngularRate(MAX_ANGULAR_RATE)
          .withChassisSpeedDeadband(
              CHASSIS_TRANSLATION_SPEED_THRESHOLD, CHASSIS_ROTATION_SPEED_THRESHOLD)
          .withGyroDevice(gyroConfiguration.canDevice)
          .withDrivetrainConstants(
              new SwerveDrivetrainConstants()
                  .withCANBusName(canBusName)
                  .withPigeon2Id(gyroConfiguration.canDevice.getDeviceID())
                  .withPigeon2Configs(gyroConfiguration.config))
          .withModuleConstants(
              new SwerveModuleConstants[] {
                moduleConfigs.frontLeftModule.getCtreModuleConstants(),
                moduleConfigs.frontRightModule.getCtreModuleConstants(),
                moduleConfigs.backLeftModule.getCtreModuleConstants(),
                moduleConfigs.backRightModule.getCtreModuleConstants()
              })
          .withOdometryStandardDevs(ENABLED_STANDARD_DEVS, DISABLED_STANDARD_DEVS)
          .withJoystickDeadband(JOYSTICK_STEER_DEADBAND, JOYSTICK_TRANSLATION_DEADBAND);
    }
  }

  /* ---- SIMULATED DRIVETRAIN ---- */
  private final class SimulatedDrivetrainConstants {
    public static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    private static DrivetrainSimConfiguration makeSimulatedDrivetrainConfiguration(
        PhysicalConfiguration physicalConfiguration) {
      return new DrivetrainSimConfiguration(SIM_LOOP_PERIOD)
          .withName(GeneralConstants.SUBSYSTEM_NAME)
          .withPhysicalConfiguration(physicalConfiguration)
          .withDriveMotorsPerModule(ProductionSwerveModuleConfigs.DRIVE_MOTORS_PER_MODULE)
          .withSteerMotorsPerModule(ProductionSwerveModuleConfigs.STEER_MOTORS_PER_MODULE);
    }
  }

  public final Pigeon2GyroConfiguration gyroConfiguration;

  public final ProductionSwerveModuleConfigs moduleConfigs;

  public final DrivetrainConfiguration drivetrainConfiguration;

  public final DrivetrainSimConfiguration simulatedDrivetrainConfiguration;

  public ProductionDrivetrainConfig(
      PhysicalConfiguration physicalConfiguration, String drivetrainCANBusName) {

    /* ---- GYRO CONFIG ---- */
    this.gyroConfiguration = GyroConstants.makeGyroConfiguration(drivetrainCANBusName);

    /* ---- MODULE CONFIGS ---- */
    this.moduleConfigs =
        new ProductionSwerveModuleConfigs(GeneralConstants.SUBSYSTEM_NAME, drivetrainCANBusName);

    /* ---- DRIVETRAIN CONFIG ---- */
    this.drivetrainConfiguration =
        DrivetrainConstants.makeDrivetrainConfiguration(
            drivetrainCANBusName, moduleConfigs, gyroConfiguration);

    /* ---- SIMULATED DRIVETRAIN CONFIG ---- */
    this.simulatedDrivetrainConfiguration =
        SimulatedDrivetrainConstants.makeSimulatedDrivetrainConfiguration(physicalConfiguration);
  }
}
