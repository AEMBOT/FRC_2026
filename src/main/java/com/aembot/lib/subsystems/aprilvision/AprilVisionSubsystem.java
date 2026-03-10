package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilVisionSubsystem extends AEMSubsystem {
  /** Maximum age (seconds) for a vision estimate to be considered valid */
  private static final double MAX_ESTIMATE_AGE_SECONDS = 0.5;

  protected final RobotState robotStateInstance;

  protected final List<Pair<AprilCameraIO, AprilVisionInputs>> camerasWithInputs =
      new ArrayList<>();

  private boolean visionActive = true;

  public AprilVisionSubsystem(RobotState robotStateInstance, AprilCameraIO... cameras) {
    super("VisionSubsystem");

    this.robotStateInstance = robotStateInstance;

    for (AprilCameraIO camera : cameras) {
      camerasWithInputs.add(Pair.of(camera, new AprilVisionInputs()));
    }

    updateNTDisabled();

    SmartDashboard.putBoolean("Vision Enabled", visionActive);
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    List<AprilCameraOutput> rawCameraOutputs = new ArrayList<>();

    // Collect all valid camera estimates
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      if (io.getConfiguration().cameraName.equals("turret")) continue;

      Logger.recordOutput(
          logPrefixStandard + "/" + io.getConfiguration().cameraName + "/CameraPosition",
          new Pose3d(robotStateInstance.getLatestFieldRobotPose())
              .plus(io.getConfiguration().getCameraPosition().minus(Pose3d.kZero)));

      io.updateInputs(inputs);

      // Check for valid estimate (use uncompensated pose for null check)
      if (inputs.coprocessorEstimationLatencyUncompensated != null && visionActive) {
        // Reject estimates that are too old
        double estimateAge = currentTime - inputs.coprocessorEstimationTimestamp;
        if (estimateAge <= MAX_ESTIMATE_AGE_SECONDS) {
          rawCameraOutputs.add(
              new AprilCameraOutput(
                  io.getConfiguration().cameraName,
                  inputs.tagID,
                  new VisionPoseEstimation(
                      inputs.coprocessorEstimationLatencyUncompensated,
                      inputs.coprocessorEstimationLatencyCompensated,
                      inputs.coprocessorEstimationStdDevs,
                      inputs.coprocessorEstimationTimestamp)));
        }
      }

      visionActive = SmartDashboard.getBoolean("Vision Enabled", true);
    }

    // Fuse multiple camera estimates using inverse-variance weighting
    List<AprilCameraOutput> fusedObservations = fuseMultiCameraEstimates(rawCameraOutputs);

    Logger.recordOutput(logPrefixStandard + "/RawCameraCount", rawCameraOutputs.size());
    Logger.recordOutput(logPrefixStandard + "/FusedObservationCount", fusedObservations.size());

    robotStateInstance.setApriltagObservations(fusedObservations);

    updateLog();
    for (AprilCameraOutput observation : robotStateInstance.getAprilTagObservations()) {
      Logger.recordOutput(
          logPrefixStandard + "/VisionEstimatedRobotPose", observation.estimatedPose());
    }

    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - currentTime) * 1000);
  }

  /**
   * Fuse multiple camera estimates into a single estimate using inverse-variance weighting. This
   * prevents camera "fighting" where multiple cameras with different estimates cause pose
   * oscillation.
   */
  private List<AprilCameraOutput> fuseMultiCameraEstimates(List<AprilCameraOutput> rawOutputs) {
    if (rawOutputs.isEmpty()) {
      return rawOutputs;
    }

    if (rawOutputs.size() == 1) {
      return rawOutputs; // No fusion needed
    }

    // Group estimates by similar timestamps (within 50ms)
    // For now, fuse all estimates into one since they're from the same periodic cycle

    double weightedX = 0;
    double weightedY = 0;
    double totalWeightX = 0;
    double totalWeightY = 0;
    double latestTimestamp = 0;
    double minXStdDev = Double.MAX_VALUE;
    double minYStdDev = Double.MAX_VALUE;

    for (AprilCameraOutput output : rawOutputs) {
      Pose2d pose = output.estimatedPose().latencyUncompensatedPose();
      OdometryStandardDevs stdDevs = output.estimatedPose().stdDevs();

      if (pose == null || stdDevs == null) continue;

      // Inverse variance weighting: weight = 1 / variance = 1 / stddev^2
      double xVariance = stdDevs.xStdDev() * stdDevs.xStdDev();
      double yVariance = stdDevs.yStdDev() * stdDevs.yStdDev();

      // Prevent division by zero
      if (xVariance < 1e-6) xVariance = 1e-6;
      if (yVariance < 1e-6) yVariance = 1e-6;

      double weightX = 1.0 / xVariance;
      double weightY = 1.0 / yVariance;

      weightedX += pose.getX() * weightX;
      weightedY += pose.getY() * weightY;
      totalWeightX += weightX;
      totalWeightY += weightY;

      // Track minimum stddevs for fused estimate
      minXStdDev = Math.min(minXStdDev, stdDevs.xStdDev());
      minYStdDev = Math.min(minYStdDev, stdDevs.yStdDev());

      // Use latest timestamp
      latestTimestamp = Math.max(latestTimestamp, output.estimatedPose().timestampSeconds());
    }

    if (totalWeightX < 1e-6 || totalWeightY < 1e-6) {
      return rawOutputs; // Fallback if weighting failed
    }

    // Calculate fused position
    double fusedX = weightedX / totalWeightX;
    double fusedY = weightedY / totalWeightY;

    // Use first camera's rotation (they should all be using gyro anyway)
    Pose2d firstPose = rawOutputs.get(0).estimatedPose().latencyUncompensatedPose();
    Pose2d fusedPose = new Pose2d(fusedX, fusedY, firstPose.getRotation());

    // Fused stddev is lower than any individual (benefit of multiple measurements)
    // stddev_fused = 1 / sqrt(sum of 1/variance)
    double fusedXStdDev = 1.0 / Math.sqrt(totalWeightX);
    double fusedYStdDev = 1.0 / Math.sqrt(totalWeightY);

    OdometryStandardDevs fusedStdDevs =
        new OdometryStandardDevs(fusedXStdDev, fusedYStdDev, Double.MAX_VALUE);

    Logger.recordOutput(logPrefixStandard + "/FusedPose", fusedPose);
    Logger.recordOutput(logPrefixStandard + "/FusedXStdDev", fusedXStdDev);
    Logger.recordOutput(logPrefixStandard + "/FusedYStdDev", fusedYStdDev);

    // Return single fused estimate
    return List.of(
        new AprilCameraOutput(
            "fused",
            -1, // No single tag ID for fused
            new VisionPoseEstimation(fusedPose, fusedPose, fusedStdDevs, latestTimestamp)));
  }

  public Command createKillVisionCommand() {
    return new InstantCommand(() -> visionActive = false);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {

    Logger.recordOutput(standardPrefix + "/VisionActive", visionActive);

    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      Logger.processInputs(inputPrefix + "/" + io.getConfiguration().toString(), inputs);
    }
  }

  private void updateNTDisabled() {
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraPair : camerasWithInputs) {
      AprilCameraIO camera = cameraPair.getFirst();
      camera.updateNetworkTablesForDisabled();
    }
  }

  private void updateNTEnabled() {
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraPair : camerasWithInputs) {
      AprilCameraIO camera = cameraPair.getFirst();
      camera.updateNetworkTablesForEnabled();
    }
  }

  /**
   * DO NOT CALL PERIODICALLY
   *
   * @return A command that update network tables with values to use while disabled
   */
  public Command updateNTDisabledCommand() {
    return new InstantCommand(this::updateNTDisabled);
  }

  /**
   * DO NOT CALL PERIODICALLY
   *
   * @return A command that update network tables with values to use while enabled
   */
  public Command updateNTEnabledCommand() {
    return new InstantCommand(this::updateNTEnabled);
  }
}
