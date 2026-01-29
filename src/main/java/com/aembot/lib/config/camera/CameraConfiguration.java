package com.aembot.lib.config.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Properties representing the configuration of a camera */
public class CameraConfiguration {

  /**
   * Camera location enum that stores both a user described camera location as well as it's true
   * pose3d.
   *
   * <p>Note: Rotation2d uses radians.
   *
   * <p>All relative directions (e.g. right and left) are from the perspective of looking from the
   * rear of the robot to the front. Where the rear and front of the robot are must be agreed upon,
   * thought it is often pretty clear.
   */
  public enum Location {
    FRONT_RIGHT("FrontRight"),
    FRONT_LEFT("FrontLeft"),
    BACK_LEFT("BackLeft"),
    BACK_RIGHT("BackRight"),
    UNCONFIGURED("Unconfigured"),
    ;

    /** Human readable name for the camera location */
    public final String LocationName;

    /** Current post of the camera relative to robot center, i.e. (0, 0, 0) */
    public Pose3d CameraPose;

    /**
     * Translation representing the position of the current camera as an offset from the center of
     * the robot.
     */
    public Translation2d TranslationToRobotCenter;

    /** Rotation 2d representing the mounting yaw rotation of the camera on the robot. */
    public Rotation2d MountingYaw;

    private Location(String name) {
      this.LocationName = name;
    }

    /**
     * Retrieve the name of the location as a string
     *
     * @return Stringified version of the camera location name
     */
    @Override
    public String toString() {
      return LocationName;
    }

    /**
     * Update the current pose of a given camera location relative to the center of the robot.
     *
     * @param pose The new pose3d that we wish to have the camera located at.
     * @return This enum, for chaining.
     */
    public Location withCameraPose(Pose3d pose) {
      this.CameraPose = pose;

      this.MountingYaw = Rotation2d.fromRadians(pose.getRotation().getZ());
      this.TranslationToRobotCenter = pose.getTranslation().toTranslation2d();
      return this;
    }
  }

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

  // Location of this camera
  public Location CameraLocation = Location.UNCONFIGURED;

  // Type of this camera
  public Type CameraType = Type.UNCONFIGURED;

  // Resolution of this camera
  public Resolution CameraResolution = Resolution._1280x960;

  // FOV of this camera
  public FOV CameraFov = FOV.LIMELIGHT4;

  // Ratio between the reported distance to a tag and the actual distance.
  public double CameraDistanceScalar = 1.0;

  /** Ratio between the limelights repoted angle and actual angle */
  public double CameraXRotationScalar = 1.0;

  /**
   * Pass in a location when the camera is configured
   *
   * @param location Location of the camera on the robot
   */
  public CameraConfiguration(Location location) {
    this.CameraLocation = location;
  }

  /**
   * Update the pose of this camera relative to the center of the robot
   *
   * @param pose The pose of this camera.
   * @return reference to this object for chaining.
   */
  public CameraConfiguration withPose(Pose3d pose) {
    this.CameraLocation.withCameraPose(pose);
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
    return CameraLocation.toString() + "_" + CameraType.toString();
  }
}
