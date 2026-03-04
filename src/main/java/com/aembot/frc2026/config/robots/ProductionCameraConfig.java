package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ProductionCameraConfig {
  private static final double SIM_CAMERA_FPS = 120;

  private static final double SIM_CAMERA_LATENCY_MS = 5;

  private static final double SIM_CAMERA_LATENCY_STDDEV_MS = 1;

  public final CameraConfiguration cameraConfigTest =
      CameraConfiguration.makeLimelight4Config("test")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0))));
  public final SimulatedCameraConfiguration simConfigTest =
      new SimulatedCameraConfiguration(cameraConfigTest)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS);

  public final List<CameraConfiguration> cameraConfigurations = List.of(cameraConfigTest);

  public final List<SimulatedCameraConfiguration> simConfigurations = List.of(simConfigTest);
}
