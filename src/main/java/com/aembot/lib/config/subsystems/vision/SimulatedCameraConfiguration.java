package com.aembot.lib.config.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

public class SimulatedCameraConfiguration {
  public final SimCameraProperties simCameraProperties = new SimCameraProperties();

  public final CameraConfiguration cameraConfiguration;

  /** The photonvision pose estimation strategy to be used in sim. */
  public PoseStrategy estimationStrategy = PoseStrategy.CONSTRAINED_SOLVEPNP;

  /**
   * Standard deviation of translational noise (meters) to add to pose estimates at 1 meter
   * distance. Scales with distance squared. Set to 0 to disable.
   */
  public double poseNoiseTranslationStdDev = 0.0;

  /**
   * Standard deviation of rotational noise (radians) to add to pose estimates at 1 meter distance.
   * Scales with distance squared. Set to 0 to disable.
   */
  public double poseNoiseRotationStdDev = 0.0;

  /**
   * Additional random latency variation (milliseconds) to add on top of PhotonVision's latency.
   * This simulates frame-to-frame processing time variation.
   */
  public double latencyVariationMs = 0.0;

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
   * @param avgErrorPixels The avg error of the camera in pixels
   * @param stddevErrorPixels The std deviations of the randomized error of the camera in pixels
   * @return this simulated camera configuration for chaining
   */
  public SimulatedCameraConfiguration withCalibrationError(
      double avgErrorPixels, double stddevErrorPixels) {
    simCameraProperties.setCalibError(avgErrorPixels, stddevErrorPixels);
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

  /**
   * Set the photonvions {@link PoseStrategy} to be used in sim.
   *
   * @return This {@link SimulatedCameraConfiguration} for chaining
   */
  public SimulatedCameraConfiguration withPoseEstimationStrategy(PoseStrategy strategy) {
    this.estimationStrategy = strategy;
    return this;
  }

  /**
   * Set Gaussian noise to add to pose estimates. Noise scales with distance squared.
   *
   * @param translationStdDevMeters Standard deviation of XY noise (meters) at 1 meter distance
   * @param rotationStdDevRadians Standard deviation of yaw noise (radians) at 1 meter distance
   * @return This {@link SimulatedCameraConfiguration} for chaining
   */
  public SimulatedCameraConfiguration withPoseNoise(
      double translationStdDevMeters, double rotationStdDevRadians) {
    this.poseNoiseTranslationStdDev = translationStdDevMeters;
    this.poseNoiseRotationStdDev = rotationStdDevRadians;
    return this;
  }

  /**
   * Set additional latency variation to simulate frame-to-frame processing time differences.
   *
   * @param stdDevMs Standard deviation of latency variation in milliseconds
   * @return This {@link SimulatedCameraConfiguration} for chaining
   */
  public SimulatedCameraConfiguration withLatencyVariation(double stdDevMs) {
    this.latencyVariationMs = stdDevMs;
    return this;
  }

  @Override
  public String toString() {
    return cameraConfiguration.toString();
  }
}
