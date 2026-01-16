package com.aembot.lib.config.subsystems.drive;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class DrivetrainConfiguration {
  public String configurationName;

  /** m/s. Any input less than this will be set to zero. */
  public double chassisTranslationSpeedThreshold;

  /** rads/s. Any input less than this will be set to zero. */
  public double chassisRotationalSpeedThreshold;

  /**
   * The deadband on the steering (usually right) joystick. Any input less than this will be set to
   * zero.
   */
  public double steerJoystickDeadband;

  /**
   * The deadband on the driving (usually left) joystick. Any input less than this will be set to
   * zero.
   */
  public double driveJoystickDeadband;

  /** The max horizontal speed the drivetrain will drive at. Not necessarily the physical limit. */
  public double maxDriveSpeed;

  /** The max angular speed the drivetrain will drive at. Not necessarily the physical limit. */
  public double maxAngularRate;

  public CANDeviceID gyroDeviceID;
  public SwerveModuleConstants<?, ?, ?>[] ctreModuleConstants;
  public SwerveDrivetrainConstants ctreDriveConstants;

  /** Standard deviations of odometry while the robot is enabled */
  public OdometryStandardDevs enabledOdometryStandardDevs;

  /** Standard deviations of odometry while the robot is disabled */
  public OdometryStandardDevs disabledOdometryStandardDevs;

  public DrivetrainConfiguration() {}

  public DrivetrainConfiguration withName(String name) {
    this.configurationName = name;
    return this;
  }

  public DrivetrainConfiguration withGyroDevice(CANDeviceID device) {
    this.gyroDeviceID = device;
    return this;
  }

  /**
   * Set the deadband for chassis speeds given to the drivetrain
   *
   * @param minTranslationMS m/s. Any input less than this will be set to zero.
   * @param minRotationRadS rads/s. Any input less than this will be set to zero.
   * @return self for chaining
   */
  public DrivetrainConfiguration withChassisSpeedDeadband(
      double minTranslationMS, double minRotationRadS) {
    this.chassisTranslationSpeedThreshold = minTranslationMS;
    this.chassisRotationalSpeedThreshold = minRotationRadS;
    return this;
  }

  public DrivetrainConfiguration withModuleConstants(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    this.ctreModuleConstants = moduleConstants;
    return this;
  }

  public DrivetrainConfiguration withDrivetrainConstants(SwerveDrivetrainConstants constants) {
    this.ctreDriveConstants = constants;
    return this;
  }

  public DrivetrainConfiguration withMaxDriveSpeed(double speed) {
    this.maxDriveSpeed = speed;
    return this;
  }

  public DrivetrainConfiguration withMaxAngularRate(double rate) {
    this.maxAngularRate = rate;
    return this;
  }

  /**
   * Set the deadband on the joysticks.
   *
   * @param steerDeadband The deadband on the steering (usually right) joystick. Any input less than
   *     this will be set to zero.
   * @param driveDeadband The deadband on the driving (usually left) joystick. Any input less than
   *     this will be set to zero.
   * @return self for chaining
   */
  public DrivetrainConfiguration withJoystickDeadband(double steerDeadband, double driveDeadband) {
    this.steerJoystickDeadband = steerDeadband;
    this.driveJoystickDeadband = driveDeadband;
    return this;
  }

  /**
   * Set stddevs for the drivetrain odometry
   *
   * @param enabledStandardDevs Standard deviations of odometry while the robot is enabled
   * @param disabledStandardDevs Standard deviations of odometry while the robot is disabled
   * @return self for chaining
   */
  public DrivetrainConfiguration withOdometryStandardDevs(
      OdometryStandardDevs enabledStandardDevs, OdometryStandardDevs disabledStandardDevs) {
    this.enabledOdometryStandardDevs = enabledStandardDevs;
    this.disabledOdometryStandardDevs = disabledStandardDevs;
    return this;
  }
}
