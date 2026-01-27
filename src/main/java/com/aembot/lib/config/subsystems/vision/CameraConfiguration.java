package com.aembot.lib.config.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

public class CameraConfiguration {
  /** The type of camera */
  public enum Type {
    LIMELIGHT("limelight"),
    ;

    /** Name of the camera type. Corresponds to the NetworkTables key */
    public final String typeName;

    private Type(String name) {
      this.typeName = name;
    }

    @Override
    public String toString() {
      return typeName;
    }
  }

  public enum Resolution {
    P1280x960(1280, 960),
    P1280x720(1280, 720),
    P640x480(640, 480),
    ;

    public final int widthPixels;
    public final int heightPixels;

    private Resolution(int widthPixels, int heightPixels) {
      this.widthPixels = widthPixels;
      this.heightPixels = heightPixels;
    }
  }

  public enum FOV {
    LIMELIGHT4(82, 56.2),
    ;

    public final double horizontalDegrees;
    public final double verticalDegrees;

    private FOV(double horizontalDegrees, double verticalDegrees) {
      this.horizontalDegrees = horizontalDegrees;
      this.verticalDegrees = verticalDegrees;
    }
  }

  /**
   * The name of the camera. For Limelights, this should be the hostname / nickname of the camera
   */
  public final String cameraName;

  public final Type cameraType;

  /**
   * The origin of this camera's mechanism relative to the center of the robot's chassis on the
   * floor. For cameras mounted to the robot chassis, the default (<0,0,0> with no rotation) will be
   * suitable.
   *
   * <p>An example use case of this field is with a turret:
   *
   * <pre>
   * {@code config.mechanismOrigin =
   *    () -> new Pose3d(0, 0, 0.3, new Rotation3d(0, 0, RobotStateYearly.get().getTurretAngleRads()));}
   * </pre>
   *
   * @see #withMechanismOrigin(Supplier)
   */
  public Supplier<Pose3d> mechanismOrigin =
      () -> {
        final var defaultOrigin = new Pose3d(0, 0, 0, new Rotation3d());
        return defaultOrigin;
      };

  /**
   * The position of the camera relative to {@link #mechanismOrigin} (usually the center of the
   * robot chassis on the floor)
   *
   * @see #withCameraOffset(Pose3d)
   */
  public Transform3d cameraOffset;

  public Resolution cameraResolution;

  public FOV cameraFOV;

  /** Ratio between the reported distance to a tag and the actual distance. Defaults to 1. */
  public double cameraDistanceScalar = 1.0;

  /**
   * Comment from 254: "When limelight tx was compared to robot rotation from the gyro, it was
   * observed that they did not scale at the same rate. Assuming the Pigeon2 scales correctly, this
   * means the limelight's angle scaling is incorrect. Because the difference scaled linearly, we
   * found out the ratio between the limelight angle and robot angle and applied it to the tx
   * angle."
   */
  public double cameraXRotationScalar = 1.0;

  public CameraConfiguration(String name, Type type) {
    this.cameraName = name;
    this.cameraType = type;
  }

  /**
   * Get the position of the camera relative to the robot chassis' center on the floor. This is the preferred
   * method of getting the camera's position.
   *
   * @see #mechanismOrigin
   * @see #cameraOffset
   */
  public Pose3d getCameraPosition() {
    return mechanismOrigin.get().transformBy(cameraOffset);
  }

  /**
   * Set the origin of this camera's mechanism relative to the center of the robot's chassis on the
   * floor. For cameras mounted to the robot chassis, the default (<0,0,0> with no rotation) will be
   * suitable.
   *
   * <p>An example use case of this field is with a turret:
   *
   * <pre>
   * {@code new CameraConfiguration("turret").withMechanismOrigin(
   *    () -> new Pose3d(0, 0, 0.3, new Rotation3d(0, 0, RobotStateYearly.get().getTurretAngleRads()))
   * );}
   * </pre>
   */
  public CameraConfiguration withMechanismOrigin(Supplier<Pose3d> origin) {
    this.mechanismOrigin = origin;
    return this;
  }

  /**
   * Set the position of the camera relative to {@link #mechanismOrigin} (usually the center of the
   * robot chassis on the floor)
   */
  public CameraConfiguration withCameraOffset(Transform3d offset) {
    this.cameraOffset = offset;
    return this;
  }

  public CameraConfiguration withCameraResolution(Resolution resolution) {
    this.cameraResolution = resolution;
    return this;
  }

  public CameraConfiguration withCameraFOV(FOV fov) {
    this.cameraFOV = fov;
    return this;
  }

  /**
   * Set the ratio between actual distances and camera-reported distances. Used to account for
   * distance irregularities in vision.
   *
   * @param actual Actual distance from the camera to an apriltag
   * @param reported Distance reported by vision to an apriltag
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withCameraDistanceScalar(double actual, double reported) {
    this.cameraDistanceScalar = actual / reported;
    return this;
  }

  /**
   * Set the ratio between the gyro reported robot yaw (presumed to be actual) and vision-reported
   * robot yaw. Used to account for rotation irregularities between camera & actual
   *
   * @param gyroRotation Rotation reported by the gyro for some angle
   * @param cameraRotation Rotation reported by the camera pose estimation for the same angle
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withXRotationScalar(double gyroRotation, double cameraRotation) {
    this.cameraXRotationScalar = gyroRotation / cameraRotation;
    return this;
  }

  @Override
  public String toString() {
    return cameraName + "_" + cameraType.typeName;
  }
}
