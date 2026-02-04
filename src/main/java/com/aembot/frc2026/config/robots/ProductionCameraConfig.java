package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ProductionCameraConfig {
  private static final double SIM_CAMERA_FPS = 120;

  private static final double SIM_CAMERA_LATENCY_MS = 5;

  private static final double SIM_CAMERA_LATENCY_STDDEV_MS = 1;

  /* ---- FRONT LEFT CAM ---- */
  public final CameraConfiguration cameraConfigFrontLeft =
      CameraConfiguration.makeLimelight4Config("Turret")
          .withMechanismOrigin(
              () -> // All super guesstimated placeholder
              new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(-3),
                          Units.inchesToMeters(0),
                          Units.inchesToMeters(6)),
                      new Rotation3d(0, 0, Units.degreesToRadians(15))))
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(3), Units.inchesToMeters(0), Units.inchesToMeters(3)),
                  new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(0), 0.0)))
          .withCameraDistanceScalar(1, 1)
          .withXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigFrontLeft =
      new SimulatedCameraConfiguration(cameraConfigFrontLeft)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCameraDistanceScalar(1.16, 1.3);

  /* ---- FRONT RIGHT CAM ---- */
  public final CameraConfiguration cameraConfigFrontRight =
      CameraConfiguration.makeLimelight4Config("FrontRight")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(9),
                      Units.inchesToMeters(-8.25),
                      Units.inchesToMeters(7.5)),
                  new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-15), 0.0)))
          .withCameraDistanceScalar(1, 1)
          .withXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigFrontRight =
      new SimulatedCameraConfiguration(cameraConfigFrontRight)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCameraDistanceScalar(1.16, 1.3);

  /* ---- BACK LEFT CAM ---- */
  public final CameraConfiguration cameraConfigBackLeft =
      CameraConfiguration.makeLimelight4Config("BackLeft")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-12),
                      Units.inchesToMeters(7.5),
                      Units.inchesToMeters(10.5)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-11.75),
                      Units.degreesToRadians(180))))
          .withCameraDistanceScalar(1, 1)
          .withXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigBackLeft =
      new SimulatedCameraConfiguration(cameraConfigBackLeft)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCameraDistanceScalar(1.16, 1.3);

  /* ---- BACK RIGHT CAM ---- */
  public final CameraConfiguration cameraConfigBackRight =
      CameraConfiguration.makeLimelight4Config("BackRight")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-12),
                      Units.inchesToMeters(-7.5),
                      Units.inchesToMeters(10.5)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-11.75),
                      Units.degreesToRadians(-180))))
          .withCameraDistanceScalar(1, 1)
          .withXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigBackRight =
      new SimulatedCameraConfiguration(cameraConfigBackRight)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCameraDistanceScalar(1.16, 1.3);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<CameraConfiguration> cameraConfigurations =
      List.of(
          cameraConfigFrontLeft,
          cameraConfigFrontRight,
          cameraConfigBackLeft,
          cameraConfigBackRight);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<SimulatedCameraConfiguration> simConfigurations =
      List.of(simConfigFrontLeft, simConfigFrontRight, simConfigBackLeft, simConfigBackRight);
}
