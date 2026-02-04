package com.aembot.lib.subsystems.vision.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class VisionPoseEstimate {

  /** Robot pose estimated based off of position */
  public final Pose2d estimatedRobotPose;

  /** Time that the estimate happened */
  public final double timestampSeconds;

  /** Number of tags used in the estimate */
  public final int numTags;

  /** Standard deviations of the measured pose */
  public final VisionStandardDeviations measurementStdDevs;

  /**
   * Create a new Vision Pose Estimate
   *
   * @param estimatedRobotPose Robot pose estimated based off of position
   * @param timestampSeconds Time that the estimate happened
   * @param numTags Number of tags used in the estimate
   * @param stdDevs Standard deviations of the measured pose
   */
  public VisionPoseEstimate(
      Pose2d estimatedRobotPose,
      double timestampSeconds,
      int numTags,
      VisionStandardDeviations stdDevs) {
    this.estimatedRobotPose = estimatedRobotPose;
    this.timestampSeconds = timestampSeconds;
    this.numTags = numTags;
    this.measurementStdDevs = stdDevs;
  }

  /**
   * Create a new VisionPoseEstimate object fused from multiple other ones
   *
   * <p>Returns empty if there is an issue fusing the poses (e.g there are no supplied estimates or
   * there is no odometry data)
   *
   * @param odometryPoseSupplier function that takes in a timestamp and returns the odometry pose at
   *     that timestamp
   * @param estimates list of VisionPoseEstimates to fuse into one
   * @return Optional fused pose estimate
   */
  public static Optional<VisionPoseEstimate> fused(
      Function<Double, Optional<Pose2d>> odometryPoseSupplier, List<VisionPoseEstimate> estimates) {

    if (estimates == null || estimates.isEmpty()) {
      return Optional.empty();
    }

    if (estimates.size() == 1) {
      return Optional.of(estimates.get(0));
    }

    double latestTimeStamp =
        estimates.stream().mapToDouble(VisionPoseEstimate::getTimestamp).max().orElse(0);

    Optional<Pose2d> latestOdometryPose = odometryPoseSupplier.apply(latestTimeStamp);
    if (latestOdometryPose.isEmpty()) {
      return Optional.empty();
    }

    double weightedXSum = 0.0;
    double weightedYSum = 0.0;

    double totalWeightX = 0.0;
    double totalWeightY = 0.0;

    int totalNumTags = 0;

    for (VisionPoseEstimate estimate : estimates) {

      Optional<Pose2d> odometryAtEstimateTime = odometryPoseSupplier.apply(estimate.getTimestamp());

      if (odometryAtEstimateTime.isEmpty()) {
        return Optional.empty();
      }

      Transform2d transform = latestOdometryPose.get().minus(odometryAtEstimateTime.get());
      Pose2d transformedPose = estimate.getEstimatedPose().transformBy(transform);

      double weightX = 1.0 / Math.pow(estimate.getStandardDeviations().getXStdDev(), 2);
      double weightY = 1.0 / Math.pow(estimate.getStandardDeviations().getYStdDev(), 2);

      weightedXSum += transformedPose.getX() * weightX;
      weightedYSum += transformedPose.getY() * weightY;

      totalWeightX += weightX;
      totalWeightY += weightY;

      totalNumTags += estimate.getNumTags();
    }

    if (totalWeightX == 0 || totalWeightY == 0) {
      return Optional.empty();
    }

    double fusedX = weightedXSum / totalWeightX;
    double fusedY = weightedYSum / totalWeightY;

    Pose2d fusedPose = new Pose2d(fusedX, fusedY, latestOdometryPose.get().getRotation());

    VisionStandardDeviations fusedStdDevs =
        new VisionStandardDeviations(
            Math.sqrt(1 / totalWeightX), Math.sqrt(1 / totalWeightY), 0, 0, 0, 0);

    return Optional.of(
        new VisionPoseEstimate(fusedPose, latestTimeStamp, totalNumTags, fusedStdDevs));
  }

  /**
   * @return Robot pose estimated based off of position
   */
  public Pose2d getEstimatedPose() {
    return estimatedRobotPose;
  }

  /**
   * @return Time that the estimate happened
   */
  public double getTimestamp() {
    return timestampSeconds;
  }

  /**
   * @return Standard deviations of the measured pose
   */
  public VisionStandardDeviations getStandardDeviations() {
    return measurementStdDevs;
  }

  /**
   * @return Number of tags used in the estimate
   */
  public int getNumTags() {
    return numTags;
  }
}
