package com.aembot.lib.config.camera;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

/** Properties representing the configuration of a camera */
public class CameraConfiguration {

  /** Enum for user described camera type */
  public enum Type {
    LIMELIGHT("Limelight"),
    UNCONFIGURED("Unconfigured"),
    ;

    /**
     * Human readable name for the camera type
     *
     * <p>additionally correspons to network table name for limelight
     */
    public final String Name;

    private Type(String name) {
      this.Name = name;
    }

    /**
     * Retrieve the name of the type as a string
     *
     * @return Stringified version of the camera location name
     */
    @Override
    public String toString() {
      return this.Name;
    }
  }

  /** Enum used to represent common camera resolutions. */
  public enum Resolution {
    _1280x960(1280, 960),
    _1280x720(1280, 720),
    _640x480(640, 480),
    ;

    public int XPixels;
    public int YPixels;

    private Resolution(int xPixels, int yPixels) {
      this.XPixels = xPixels;
      this.YPixels = yPixels;
    }
  }

  /** Enum used to represent common camera FOVs. */
  public enum FOV {
    LIMELIGHT4(82, 56.2);

    public double HorizontalDegrees;
    public double VerticalDegrees;

    private FOV(double horizontalDegrees, double verticalDegrees) {
      this.HorizontalDegrees = horizontalDegrees;
      this.VerticalDegrees = verticalDegrees;
    }
  }

  /** Name of the this camera */
  public String CameraName = "UNNAMED";

  /** Pose of this camera relative to the mechanism it is on */
  public Pose3d CameraPose = new Pose3d();

  /** Supplier of the mechanism position, only applicable if camera moves relative to the robot */
  public Supplier<Pose3d> MechanismPoseSupplier = () -> Pose3d.kZero;

  /** Supplier of the mechanism's angular velocity */
  public Supplier<Double> MechanismAngVelSupplier = () -> Double.valueOf(0);

  /** Type of this camera */
  public Type CameraType = Type.UNCONFIGURED;

  /** Resolution of this camera */
  public Resolution CameraResolution = Resolution._1280x960;

  /** FOV of this camera */
  public FOV CameraFov = FOV.LIMELIGHT4;

  /** Ratio between the reported distance to a tag and the actual distance. */
  public double CameraDistanceScalar = 1.0;

  /** Ratio between the limelights repoted angle and actual angle */
  public double CameraXRotationScalar = 1.0;

  /**
   * Pass in a location when the camera is configured
   *
   * @param location Location of the camera on the robot
   */
  public CameraConfiguration(String name) {
    this.CameraName = name;
  }

  /**
   * Update the pose of this camera relative to the mechanism it is connected to
   *
   * <p>For most cases this mechanism is just the center of the robot
   *
   * @param pose The pose of this camera.
   * @return reference to this object for chaining.
   */
  public CameraConfiguration withPose(Pose3d pose) {
    this.CameraPose = pose;
    return this;
  }

  /**
   * Give a supplier for the pose of the mechanism the camera is connected to
   *
   * <p>This is helpful when the camera moves relative to the robot, for example if it is connected
   * to a turret
   *
   * @param mechanismPoseSupplier Supplier of the mechanism pose
   * @return reference to this object for chaining
   */
  public CameraConfiguration withMechanismPoseSupplier(Supplier<Pose3d> mechanismPoseSupplier) {
    this.MechanismPoseSupplier = mechanismPoseSupplier;
    return this;
  }

  /**
   * Give a supplier for the angular velocity of the mechanism connected to this camera
   *
   * <p>This is helpful when the camera moves relative to the robot, for example if it is connected
   * to a turret
   *
   * @param mechanismAngVelSupplier Supplier of the mechanism angular velocity
   * @return reference to this object for chaining
   */
  public CameraConfiguration withMechanismAngularVelocitySupplier(
      Supplier<Double> mechanismAngVelSupplier) {
    this.MechanismAngVelSupplier = mechanismAngVelSupplier;
    return this;
  }

  /**
   * Update type of camera that is used here
   *
   * @param type What type of camera is in use in this configuration
   * @return reference to this object for chaining
   */
  public CameraConfiguration withType(Type type) {
    this.CameraType = type;
    return this;
  }

  /**
   * Update resolution of this camera
   *
   * @param resolution what resolution of the camera we are using
   * @return reference to this object for chaining
   */
  public CameraConfiguration withResolution(Resolution resolution) {
    this.CameraResolution = resolution;
    return this;
  }

  /**
   * Update FOV of this camera
   *
   * @param fov What the fov of the camera is
   * @return reference to this object for chaining
   */
  public CameraConfiguration withFOV(FOV fov) {
    this.CameraFov = fov;
    return this;
  }

  /**
   * Configure the ratio that needs to be provided to account for distance irregularities in vision
   *
   * <p>Units of both parameters must be identical
   *
   * @param actual The actual distance to some landmark (e.g. april tag)
   * @param reported The distance reported by vision to some landmark (e.g. apriltag)
   * @return reference to this object for chaining
   */
  public CameraConfiguration withCalculatedDistanceScalar(double actual, double reported) {
    this.CameraDistanceScalar = actual / reported;
    return this;
  }

  /**
   * Configure the ratio that needs to be provided to account for x rotation irregularities between
   * vision and gyro
   *
   * <p>Units of both parameters must be identical
   *
   * @param gyroChange The rotation reported by the gyro for some angle
   * @param cameraChange The rotation reported by the camera for the same angle
   * @return reference to this object for chaining
   */
  public CameraConfiguration withCalculatedXRotationScalar(double gyroChange, double cameraChange) {
    this.CameraXRotationScalar = gyroChange / cameraChange;
    return this;
  }

  @Override
  public String toString() {
    return CameraName;
  }

  /**
   * create a new camera config for limelight4
   *
   * @param name name of the camera
   * @return the new config
   */
  public static CameraConfiguration limelight4Config(String name) {
    return new CameraConfiguration(name)
        .withResolution(Resolution._1280x720)
        .withFOV(FOV.LIMELIGHT4);
  }
}
