package com.aembot.lib.config.robot;

import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation3d;

/** Configuration for the Pigeon2 used on the robot */
public class Pigeon2GyroConfiguration {

  /**
   * Number of degrees per reported rotation of the gyro. To account for gyro possibly dilating
   * values
   *
   * <p>To determine, rotate robot a known number of degrees and observe gyro reported yaw. Set this
   * value to (realYaw / reportedYaw)
   */
  private double gyroYawScalarDegrees = 1.0;

  /** Rotation that the gyro is mounted at */
  private Rotation3d mountRotation = new Rotation3d(0, 0, 0);

  /** CAN Device ID associated with the gyro */
  public CANDeviceID canDevice = null;

  /** Configuration for the Pigeon2 that is to be used with the swerve drive */
  public final Pigeon2Configuration config = new Pigeon2Configuration();

  public Pigeon2GyroConfiguration() {
    applyGyroYawScalar();
    applyGyroMountRotation();
  }

  /**
   * Get the number of degrees per reported rotation of the gyro, to account for gyro possibly
   * dilating values.
   */
  public double getGyroYawScalar() {
    return gyroYawScalarDegrees;
  }

  /**
   * Set the number of degrees per reported rotation of the gyro. To account for gyro possibly
   * dilating values
   *
   * <p>To determine, rotate robot a known number of degrees and observe gyro reported yaw. Set this
   * value to (realYaw / reportedYaw)
   */
  public Pigeon2GyroConfiguration withGyroYawScalar(double error) {
    this.gyroYawScalarDegrees = error;
    applyGyroYawScalar();

    return this;
  }

  /** Apply gyroYawScalarDegrees to the underlying Pigeon 2 config */
  private void applyGyroYawScalar() {
    this.config.withGyroTrim(new GyroTrimConfigs().withGyroScalarZ(gyroYawScalarDegrees));
  }

  public Rotation3d getGyroMountRotation() {
    return mountRotation;
  }

  public Pigeon2GyroConfiguration withGyroMountRotation(Rotation3d mountRot) {
    this.mountRotation = mountRot;
    applyGyroMountRotation();
    return this;
  }

  private void applyGyroMountRotation() {
    this.config.withMountPose(
        new MountPoseConfigs()
            .withMountPoseRoll(Math.toDegrees(mountRotation.getX()))
            .withMountPosePitch(Math.toDegrees(mountRotation.getY()))
            .withMountPoseYaw(Math.toDegrees(mountRotation.getZ())));
  }

  public Pigeon2GyroConfiguration withCANDevice(CANDeviceID device) {
    this.canDevice = device;
    return this;
  }
}
