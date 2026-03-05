package com.aembot.lib.config.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * The translational standard deviations of the camera with 1 tag 1 meter away from the camera.
   * This will be used as {@code baseline * (avgDistance^2 / numTags) * methodSpecificScalar}
   */
  public double baselineTranslationalStdDev = 0.02;

  /**
   * The rotational standard deviations of the camera with 1 tag 1 meter away from the camera This
   * will be used as {@code baseline * (avgDistance^2 / numTags) * methodSpecificScalar}
   */
  public double baselineAngularStdDev = Double.MAX_VALUE;

  /**
   * The throttle value when the camera is disabled. What this represents is
   * implementation-specific. On LL, this is the number of frames skipped
   */
  public double disabledThrottleValue = 0;

  /**
   * The throttle value when the camera is enabled. What this represents is implementation-specific.
   * On LL, this is the number of frames skipped
   */
  public double enabledThrottledValue = 0;

  public CameraConfiguration(String name, Type type) {
    this.cameraName = name;
    this.cameraType = type;
  }

  /**
   * Get the position of the camera relative to the robot chassis' center on the floor. This is the
   * preferred method of getting the camera's position.
   *
   * @see #mechanismOrigin
   * @see #cameraOffset
   */
  public Pose3d getCameraPosition() {
    return mechanismOrigin.get().transformBy(cameraOffset);
  }

  /** Get the pitch of the camera */
  public Rotation2d getCameraPitch() {
    Rotation3d yawAccountedPitch =
        getCameraPosition()
            .getRotation()
            .rotateBy(new Rotation3d(0, 0, -getCameraPosition().getRotation().getZ()));

    return Rotation2d.fromRadians(yawAccountedPitch.getY());
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
   * Set the translational standard deviations of the camera with 1 tag 1 meter away from the
   * camera. This will be used as {@code baseline * (avgDistance^2 / numTags) *
   * methodSpecificScalar}
   *
   * <p><strong>Note:</strong> some camera implementations (ie. Limelight hardware) might discard
   * this value in favor of its own std dev calculation method (ie. Limelight-provided stddevs)
   *
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withBaselineTranslationalStdDev(double stdDevMeters) {
    this.baselineTranslationalStdDev = stdDevMeters;
    return this;
  }

  /**
   * Set the angular standard deviations of the camera with 1 tag 1 meter away from the camera. This
   * will be used as {@code baseline * (avgDistance^2 / numTags) * methodSpecificScalar}
   *
   * <p><strong>Note:</strong> some camera implementations (ie. Limelight hardware) might discard
   * this value in favor of its own std dev calculation method (ie. Limelight-provided stddevs)
   *
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withBaselineAngularStdDev(double stdDevRads) {
    this.baselineAngularStdDev = stdDevRads;
    return this;
  }

  /**
   * Set the translational and angular standard deviations of the camera with 1 tag 1 meter away
   * from the camera. This will be used as {@code baseline * (avgDistance^2 / numTags) *
   * methodSpecificScalar}.
   *
   * <p><strong>Note:</strong> some camera implementations (ie. Limelight hardware) might discard
   * this value in favor of its own std dev calculation method (ie. Limelight-provided stddevs)
   *
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withBaslineStdDev(
      double baselineTranslationalStdDev, double baselineAngularStdDev) {
    return this.withBaselineTranslationalStdDev(baselineTranslationalStdDev)
        .withBaselineAngularStdDev(baselineAngularStdDev);
  }

  /**
   * Set the throttle value when the camera is disabled. What this represents is
   * implementation-specific. On LL, this is the number of frames skipped
   *
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withDisabledThrottleValue(double disabledThrottleValue) {
    this.disabledThrottleValue = disabledThrottleValue;
    return this;
  }

  /**
   * Set the throttle value when the camera is enabled. What this represents is
   * implementation-specific. On LL, this is the number of frames skipped
   *
   * @return this {@link CameraConfiguration} for chaining
   */
  public CameraConfiguration withEnabledThrottleValue(double enabledThrottledValue) {
    this.enabledThrottledValue = enabledThrottledValue;
    return this;
  }

  @Override
  public String toString() {
    return cameraName + "_" + cameraType.typeName;
  }

  /* ---- FACTORY METHODS ---- */
  public static CameraConfiguration makeLimelight4Config(String name) {
    return new CameraConfiguration(name, Type.LIMELIGHT)
        .withCameraResolution(Resolution.P1280x960)
        .withCameraFOV(FOV.LIMELIGHT4)
        .withBaslineStdDev(0.02, Double.MAX_VALUE) // Values yoinked from 2481
        .withDisabledThrottleValue(2) // TODO
        .withEnabledThrottleValue(2);
  }
}
