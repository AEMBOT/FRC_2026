package com.aembot.frc2026.config.robots.production;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import com.aembot.lib.config.robot.PhysicalConfiguration;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ProductionConfig extends RobotConfiguration {

  private static final String ROBOT_NAME = "Timothy";

  private static final PhysicalConfiguration PHYSICAL_CONFIGURATION =
      new PhysicalConfiguration()
          .withRobotWeightPounds(156)
          .withWheelBaseLengthM(Units.inchesToMeters(22.75))
          .withWheelTrackWidthM(Units.inchesToMeters(22.75))
          .withBumperLengthM(Units.inchesToMeters(35.625))
          .withBumperWidthM(Units.inchesToMeters(35.625))
          .withWheelCoefficientOfFriction(1.2);

  private static final ProductionCameraConfig CAMERA_CONFIG = new ProductionCameraConfig();

  @Override
  public String getRobotName() {
    return ROBOT_NAME;
  }

  @Override
  public List<CameraConfiguration> getCameraConfigurations() {
    return CAMERA_CONFIG.cameraConfigurations;
  }

  @Override
  public List<SimulatedCameraConfiguration> getSimulatedCameraConfigurations() {
    return CAMERA_CONFIG.simConfigurations;
  }
}
