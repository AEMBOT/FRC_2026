package com.aembot.lib.config.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.SimCameraProperties;

/** Properties representing the configuration of a camera in sim */
public class SimulatedCameraConfiguration {

  /** Properties of the simulated camera */
  public final SimCameraProperties kSimCameraProperties = new SimCameraProperties();

  /** Camera configuration of the camera that is being simulated */
  public final CameraConfiguration kCameraConfiguration;

  /**
   * Create a new simulated camera config
   *
   * @param cameraConfig Configuration of the camera to simulate
   */
  public SimulatedCameraConfiguration(CameraConfiguration cameraConfig) {
    this.kCameraConfiguration = cameraConfig;

    //// --- Math to derive diagonal fov ---
    double hRad = Math.toRadians(this.kCameraConfiguration.CameraFov.HorizontalDegrees);
    double vRad = Math.toRadians(this.kCameraConfiguration.CameraFov.VerticalDegrees);

    double diagRad =
        2 * Math.atan(Math.sqrt(Math.pow(Math.tan(hRad), 2) + Math.pow(Math.tan(vRad), 2)));

    Rotation2d diagFov = new Rotation2d(diagRad);
    //// ------------------------------------

    withSpecs(
        this.kCameraConfiguration.CameraResolution.XPixels,
        this.kCameraConfiguration.CameraResolution.YPixels,
        diagFov);
  }

  /**
   * Configure the simulated camera with the provided specifications
   *
   * @param xPixels Number of pixels along the x axis
   * @param yPixels Number of pixels along the y axis
   * @param diagFOV Diagonal fov of the camera
   * @return reference to this object for chaining
   */
  public SimulatedCameraConfiguration withSpecs(int xPixels, int yPixels, Rotation2d diagFOV) {
    kSimCameraProperties.setCalibration(xPixels, yPixels, diagFOV);
    return this;
  }

  /**
   * Configure the simulated camera to have noise, in pixel amounts
   *
   * @param xPixelNoise How much error in piexls to add to the sim along the x axis
   * @param yPixelNoise How much error in piexls to add to the sim along the y axis
   * @return reference to this object for chaining
   */
  public SimulatedCameraConfiguration withCameraNoise(double xPixelNoise, double yPixelNoise) {
    kSimCameraProperties.setCalibError(xPixelNoise, yPixelNoise);
    return this;
  }

  /**
   * Configure the simulated camera to have a set framerate
   *
   * @param fps Frames per second for simulated camera
   * @return reference to this object for chaining
   */
  public SimulatedCameraConfiguration withFramerate(double fps) {
    kSimCameraProperties.setFPS(fps);
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
  public SimulatedCameraConfiguration withCalculatedDistanceScalar(double actual, double reported) {
    kCameraConfiguration.withCalculatedDistanceScalar(actual, reported);
    return this;
  }

  /**
   * Configure the simulated camera to have latency
   *
   * @param latencyMS Latency of the camera in milliseconds
   * @param latencyStdDevMS Standard deviation of latency in millisecond
   * @return referene to this object for chaining
   */
  public SimulatedCameraConfiguration withCameraLatency(double latencyMS, double latencyStdDevMS) {
    kSimCameraProperties.setAvgLatencyMs(latencyMS);
    kSimCameraProperties.setLatencyStdDevMs(latencyStdDevMS);
    return this;
  }

  @Override
  public String toString() {
    return kCameraConfiguration.toString();
  }
}
