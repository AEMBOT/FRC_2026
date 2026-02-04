package com.aembot.frc2026.config;

import com.aembot.frc2026.config.robots.production.ProductionConfig;
import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import java.util.List;

public abstract class RobotConfiguration {
  /**
   * Get the name of the robot has a human readable string
   *
   * @return Robot name as a string
   */
  public abstract String getRobotName();

  /**
   * Get configuration details of all the cameras for this robot
   *
   * @return List of {@link CameraConfiguration}s of the given robot
   */
  public abstract List<CameraConfiguration> getCameraConfigurations();

  /**
   * Get configuration details of all the simulated cameras for this robot
   *
   * @return List of {@link SimulatedCameraConfiguration}s of the given robot
   */
  public abstract List<SimulatedCameraConfiguration> getSimulatedCameraConfigurations();

  /**
   * retrieve the correct robot constants based on the given robot identification
   *
   * @param identification Robot identification representing what robot is currently in use
   * @return The desired configuration that should be utilized with this robot
   */
  public static RobotConfiguration getRobotConstants(RobotIDYearly identification) {
    switch (identification) {
      case PRODUCTION:
      default:
        return new ProductionConfig();
    }
  }
}
