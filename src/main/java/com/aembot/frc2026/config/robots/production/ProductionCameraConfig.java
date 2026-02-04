package com.aembot.frc2026.config.robots.production;

import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ProductionCameraConfig {

  private static final double SIM_CAMERA_FPS = 120;

  private static final double SIM_CAMERA_LATENCY_MS = 5;

  private static final double SIM_CAMERA_LATENCY_STDDEV_MS = 1;

  // ALL VALUES ARE COMPLETELY ARBITRARY
  public final CameraConfiguration cameraConfigTurret =
      CameraConfiguration.limelight4Config("Turret")
          .withMechanismPoseSupplier(
              () ->
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(-3),
                          Units.inchesToMeters(0),
                          Units.inchesToMeters(6)),
                      new Rotation3d(
                          Units.degreesToRadians(0),
                          Units.degreesToRadians(0),
                          Units.degreesToRadians(180))))
          .withPose(
              new Pose3d(
                  new Translation3d(
                      Units.inchesToMeters(2), Units.inchesToMeters(0), Units.inchesToMeters(4)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(30),
                      Units.degreesToRadians(0))))
          .withCalculatedDistanceScalar(1, 1)
          .withCalculatedXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigTurret =
      new SimulatedCameraConfiguration(cameraConfigTurret)
          .withFramerate(SIM_CAMERA_FPS)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCalculatedDistanceScalar(1.16, 1.3);

  public final CameraConfiguration cameraConfigFront =
      CameraConfiguration.limelight4Config("Front")
          .withPose(
              new Pose3d(
                  new Translation3d(
                      Units.inchesToMeters(6), Units.inchesToMeters(0), Units.inchesToMeters(8)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(30),
                      Units.degreesToRadians(0))))
          .withCalculatedDistanceScalar(1, 1)
          .withCalculatedXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigFront =
      new SimulatedCameraConfiguration(cameraConfigFront)
          .withFramerate(SIM_CAMERA_FPS)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCalculatedDistanceScalar(1.16, 1.3);

  public final CameraConfiguration cameraConfigLeft =
      CameraConfiguration.limelight4Config("Left")
          .withPose(
              new Pose3d(
                  new Translation3d(
                      Units.inchesToMeters(0), Units.inchesToMeters(6), Units.inchesToMeters(8)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(30),
                      Units.degreesToRadians(90))))
          .withCalculatedDistanceScalar(1, 1)
          .withCalculatedXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigLeft =
      new SimulatedCameraConfiguration(cameraConfigLeft)
          .withFramerate(SIM_CAMERA_FPS)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCalculatedDistanceScalar(1.16, 1.3);

  public final CameraConfiguration cameraConfigRight =
      CameraConfiguration.limelight4Config("Left")
          .withPose(
              new Pose3d(
                  new Translation3d(
                      Units.inchesToMeters(0), Units.inchesToMeters(-6), Units.inchesToMeters(8)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(30),
                      Units.degreesToRadians(-90))))
          .withCalculatedDistanceScalar(1, 1)
          .withCalculatedXRotationScalar(1, -1);

  public final SimulatedCameraConfiguration simConfigRight =
      new SimulatedCameraConfiguration(cameraConfigRight)
          .withFramerate(SIM_CAMERA_FPS)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withCalculatedDistanceScalar(1.16, 1.3);

  public final List<CameraConfiguration> cameraConfigurations =
      List.of(cameraConfigTurret, cameraConfigFront, cameraConfigLeft, cameraConfigRight);

  public final List<SimulatedCameraConfiguration> simConfigurations =
      List.of(simConfigTurret, simConfigFront, simConfigLeft, simConfigRight);
}
