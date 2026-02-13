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

  /* ---- TURRET CAM ---- */
  public final CameraConfiguration cameraConfigTurret =
      CameraConfiguration.makeLimelight4Config("Turret")
          .withMechanismOrigin(
              () ->
                  new Pose3d(
                      new Translation3d(
                          Units.inchesToMeters(-5.3125),
                          Units.inchesToMeters(0),
                          Units.inchesToMeters(0)),
                      new Rotation3d(
                          0,
                          0,
                          Units.degreesToRadians(
                              0)))) // TODO grab actual turret rot from robot state
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(6.0625),
                      Units.inchesToMeters(0),
                      Units.inchesToMeters(18.75)),
                  new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-27), 0.0)));

  public final SimulatedCameraConfiguration simConfigTurret =
      new SimulatedCameraConfiguration(cameraConfigTurret)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS);

  /* ---- RIGHT CAM ---- */
  public final CameraConfiguration cameraConfigRight =
      CameraConfiguration.makeLimelight4Config("Right")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-3),
                      Units.inchesToMeters(-13.5),
                      Units.inchesToMeters(15.132)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(-90))));

  public final SimulatedCameraConfiguration simConfigRight =
      new SimulatedCameraConfiguration(cameraConfigRight)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS);

  /* ---- LEFT CAM ---- */
  public final CameraConfiguration cameraConfigLeft =
      CameraConfiguration.makeLimelight4Config("Left")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-3),
                      Units.inchesToMeters(13.5),
                      Units.inchesToMeters(15.132)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(90))));
  public final SimulatedCameraConfiguration simConfigLeft =
      new SimulatedCameraConfiguration(cameraConfigLeft)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS);

  /* ---- BACK CAM ---- */
  public final CameraConfiguration cameraConfigBack =
      CameraConfiguration.makeLimelight4Config("Back")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-13),
                      Units.inchesToMeters(0),
                      Units.inchesToMeters(12.2)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(-180))));

  public final SimulatedCameraConfiguration simConfigBack =
      new SimulatedCameraConfiguration(cameraConfigBack)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<CameraConfiguration> cameraConfigurations =
      List.of(cameraConfigTurret, cameraConfigRight, cameraConfigLeft, cameraConfigBack);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<SimulatedCameraConfiguration> simConfigurations =
      List.of(simConfigTurret, simConfigRight, simConfigLeft, simConfigBack);
}
