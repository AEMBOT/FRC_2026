package com.aembot.lib.config.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConfiguration {

  public enum Location {
    FRONT_RIGHT("FrontRight"),
    FRONT_LEFT("FrontLeft"),
    BACK_LEFT("BackLeft"),
    BACK_RIGHT("BackRight"),
    UNCONFIGURED("Unconfigured"),
    ;

    public final String LocationName;

    public Pose3d CameraPose;

    public Translation2d TranslationToRobotCenter;

    public Rotation2d MountingYaw;

    private Location(String name) {
      this.LocationName = name;
    }

    @Override
    public String toString() {
      return LocationName;
    }

    public Location withCameraPose(Pose3d pose) {
      this.CameraPose = pose;

      this.MountingYaw = Rotation2d.fromRadians(pose.getRotation().getZ());
      this.TranslationToRobotCenter = pose.getTranslation().toTranslation2d();
      return this;
    }
  }

  public enum Type {
    LIMELIGHT("Limelight"),
    UNCONFIGURED("Unconfigured"),
;

    public final String Name;

    private Type(String name) {
      this.Name = name;
    }

    @Override
    public String toString() {
      return this.Name;
    }
  }

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

  public enum FOV {
    LIMELIGHT4(82, 56.2);

    public double HorizontalDegrees;
    public double VerticalDegrees;

    private FOV(double horizontalDegrees, double verticalDegrees) {
      this.HorizontalDegrees = horizontalDegrees;
      this.VerticalDegrees = verticalDegrees;
    }
  }

  public Location CameraLocation = Location.UNCONFIGURED;

  public Type CameraType = Type.UNCONFIGURED;

  public Resolution CameraResolution = Resolution._1280x960;

  public FOV CameraFov = FOV.LIMELIGHT4;

  public double CameraDistanceScalar = 1.0;

  public double CameraXRotationScalar = 1.0;

  public CameraConfiguration(Location location) {
    this.CameraLocation = location;
  }

  public CameraConfiguration withPose(Pose3d pose) {
    this.CameraLocation.withCameraPose(pose);
    return this;
  }

  public CameraConfiguration withType(Type type) {
    this.CameraType = type;
    return this;
  }

  public CameraConfiguration withResolution(Resolution resolution) {
    this.CameraResolution = resolution;
    return this;
  }

  public CameraConfiguration withFOV(FOV fov) {
    this.CameraFov = fov;
    return this;
  }

  public CameraConfiguration withCalculatedDistanceScalar(double actual, double reported) {
    this.CameraDistanceScalar = actual / reported;
    return this;
  }

  public CameraConfiguration withCalculatedXRotationScalar(double gyroChange, double cameraChange) {
    this.CameraXRotationScalar = gyroChange / cameraChange;
    return this;
  }

  @Override 
  public String toString() {
    return CameraLocation.toString() + "_" + CameraType.toString();
  }
}
