package com.aembot.lib.config.subsystems.vision;

import com.aembot.lib.constants.RuntimeConstants;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.photonvision.simulation.SimCameraProperties;

public class SimulatedCameraConfiguration {
  public final SimCameraProperties simCameraProperties = new SimCameraProperties();

  public final CameraConfiguration cameraConfiguration;

  public SimulatedCameraConfiguration(CameraConfiguration cameraConfiguration) {
    this.cameraConfiguration = cameraConfiguration;

    double fovHorizontalRads =
        Units.degreesToRadians(this.cameraConfiguration.cameraFOV.horizontalDegrees / 2);
    double fovVerticalRads =
        Units.degreesToRadians(this.cameraConfiguration.cameraFOV.verticalDegrees / 2);

    double fovDiagonalRads =
        2 * Math.atan(Math.hypot(Math.tan(fovHorizontalRads), Math.tan(fovVerticalRads)));

    Rotation2d fovDiagonal = Rotation2d.fromRadians(fovDiagonalRads);

    withSpecs(
        cameraConfiguration.cameraResolution.widthPixels,
        cameraConfiguration.cameraResolution.heightPixels,
        fovDiagonal);
  }

  /**
   * Configure the simulated camera to have the given specifications. Called automatically by
   * constructor.
   *
   * @param xPixels Resolution width of the camera
   * @param yPixels Resolution height of the camera
   * @param diagFOV Diagonal FOV of the camera
   * @return this {@link SimulatedCameraConfiguration} for chaining
   */
  public SimulatedCameraConfiguration withSpecs(int xPixels, int yPixels, Rotation2d diagFOV) {
    simCameraProperties.setCalibration(xPixels, yPixels, diagFOV);
    return this;
  }

  /**
   * Configure the simulated camera to have a set framerate
   *
   * @param fps Camera frames per second
   * @return this simulated camera configuration for chaining
   */
  public SimulatedCameraConfiguration withFramerate(double fps) {
    simCameraProperties.setFPS(fps);
    return this;
  }

  /**
   * Configure the simulated camera to have noise in pixel amounts
   *
   * @param xPixelsNoise How much error in pixels should we add into the simulation along the x
   * @param yPixelsNoise How much error in pixels should we add into the simulation along the y
   * @return this simulated camera configuration for chaining
   */
  public SimulatedCameraConfiguration withCameraNoise(double xPixelNoise, double yPixelNoise) {
    simCameraProperties.setCalibError(xPixelNoise, yPixelNoise);
    return this;
  }

  /**
   * Sets the {@link CameraConfiguration#cameraDistanceScalar} of the camera's configuration while
   * running in sim. Note that this does not affect limelight's camera simulation; rather, it
   * affects the vision subsystem in the same way it would on the real robot.
   *
   * @param actual Actual distance from the camera to an apriltag
   * @param reported Distance reported by vision to an apriltag
   * @return this {@link SimulatedCameraConfiguration} for chaining
   * @see CameraConfiguration#withCameraDistanceScalar(double, double)
   */
  public SimulatedCameraConfiguration withCameraDistanceScalar(double actual, double reported) {
    if (RuntimeConstants.MODE == RuntimeMode.SIM) {
      cameraConfiguration.withCameraDistanceScalar(actual, reported);
    }
    return this;
  }

  /**
   * Set the simulated latency of the camera
   *
   * @param avgLatencyMs The average latency (from image capture to data published) in milliseconds
   *     a frame should have
   * @param latencyStdDevMs The standard deviation in milliseconds of the latency
   * @return this {@link SimulatedCameraConfiguration} for chaining
   */
  public SimulatedCameraConfiguration withCameraLatency(
      double avgLatencyMs, double latencyStdDevMs) {
    simCameraProperties.setAvgLatencyMs(avgLatencyMs);
    simCameraProperties.setLatencyStdDevMs(latencyStdDevMs);
    return this;
  }

  @Override
  public String toString() {
    return cameraConfiguration.toString();
  }
}
