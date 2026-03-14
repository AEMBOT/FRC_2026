package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
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

  // ===== STD DEV COMPUTATION CONSTANTS (tunable in replay) =====
  private static final int MEGATAG2_X_STDDEV_INDEX = 6;
  private static final int MEGATAG2_Y_STDDEV_INDEX = 7;
  private static final int EXPECTED_STDDEV_ARRAY_LENGTH = 12;
  private static final double FALLBACK_TRANSLATION_STDDEV = 0.5;

  /** Max rotation rate (rad/s) before rejecting single-tag estimates */
  private static final double MAX_OMEGA_FOR_SINGLE_TAG = Units.degreesToRadians(150);

  /** Max rotation rate (rad/s) before rejecting all estimates */
  private static final double MAX_OMEGA_FOR_ANY_TAG = Units.degreesToRadians(360);

  // Tag area thresholds (percentage of image) for quality scoring
  private static final double TAG_AREA_REJECT_THRESHOLD = 0.05;
  private static final double TAG_AREA_FAR_THRESHOLD = 0.1;
  private static final double TAG_AREA_MEDIUM_THRESHOLD = 1.0;
  private static final double TAG_AREA_GOOD_THRESHOLD = 5.0;

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
      CameraConfiguration config = io.getConfiguration();

      Logger.recordOutput(
          logPrefixStandard + "/" + config.cameraName + "/CameraPosition",
          new Pose3d(robotStateInstance.getLatestFieldRobotPose())
              .plus(config.getCameraPosition().minus(Pose3d.kZero)));

      io.updateInputs(inputs);

      // Check for valid raw estimate
      if (inputs.rawCoprocessorPose != null && inputs.tagCount > 0 && visionActive) {
        // Reject estimates that are too old
        double estimateAge = currentTime - inputs.coprocessorEstimationTimestamp;
        if (estimateAge <= MAX_ESTIMATE_AGE_SECONDS) {
          // === COMPUTE POSE AND STDDEVS (replayable) ===
          VisionPoseEstimation processed = processRawEstimate(inputs, config, config.cameraName);

          if (processed != null && processed.latencyUncompensatedPose() != null) {
            rawCameraOutputs.add(new AprilCameraOutput(config.cameraName, inputs.tagID, processed));

            // Log computed values as OUTPUTS
            Logger.recordOutput(
                logPrefixStandard + "/" + config.cameraName + "/ComputedPose",
                processed.latencyUncompensatedPose());
            Logger.recordOutput(
                logPrefixStandard + "/" + config.cameraName + "/ComputedXStdDev",
                processed.stdDevs().xStdDev());
            Logger.recordOutput(
                logPrefixStandard + "/" + config.cameraName + "/ComputedYStdDev",
                processed.stdDevs().yStdDev());
          }
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
   *
   * <p>Before fusing, older estimates are "previewed" forward to the latest timestamp using
   * odometry, so all estimates are effectively at the same point in time (254's approach).
   */
  private List<AprilCameraOutput> fuseMultiCameraEstimates(List<AprilCameraOutput> rawOutputs) {
    if (rawOutputs.isEmpty()) {
      return rawOutputs;
    }

    if (rawOutputs.size() == 1) {
      return rawOutputs; // No fusion needed
    }

    // Find the latest timestamp among all estimates
    double latestTimestamp = 0;
    for (AprilCameraOutput output : rawOutputs) {
      latestTimestamp = Math.max(latestTimestamp, output.estimatedPose().timestampSeconds());
    }

    // Get odometry pose at the latest timestamp (target time for all estimates)
    Pose2d odomAtLatest = robotStateInstance.getFieldRobotPoseForTimestamp(latestTimestamp);
    if (odomAtLatest == null) {
      // Can't preview without odometry, fall back to simple fusion
      Logger.recordOutput(logPrefixStandard + "/FusePreviewFailed", true);
      return rawOutputs;
    }

    double weightedX = 0;
    double weightedY = 0;
    double totalWeightX = 0;
    double totalWeightY = 0;

    for (AprilCameraOutput output : rawOutputs) {
      Pose2d pose = output.estimatedPose().latencyUncompensatedPose();
      OdometryStandardDevs stdDevs = output.estimatedPose().stdDevs();
      double timestamp = output.estimatedPose().timestampSeconds();

      if (pose == null || stdDevs == null) continue;

      // Preview this estimate forward to the latest timestamp using odometry
      Pose2d previewedPose = pose;
      if (timestamp < latestTimestamp) {
        Pose2d odomAtThis = robotStateInstance.getFieldRobotPoseForTimestamp(timestamp);
        if (odomAtThis != null) {
          // Compute transform from this timestamp to latest: how did the robot move?
          Transform2d odomDelta = odomAtLatest.minus(odomAtThis);
          // Apply that motion to the vision estimate
          previewedPose = pose.transformBy(odomDelta);
        }
      }

      // Inverse variance weighting: weight = 1 / variance = 1 / stddev^2
      double xVariance = stdDevs.xStdDev() * stdDevs.xStdDev();
      double yVariance = stdDevs.yStdDev() * stdDevs.yStdDev();

      // Prevent division by zero
      if (xVariance < 1e-6) xVariance = 1e-6;
      if (yVariance < 1e-6) yVariance = 1e-6;

      double weightX = 1.0 / xVariance;
      double weightY = 1.0 / yVariance;

      weightedX += previewedPose.getX() * weightX;
      weightedY += previewedPose.getY() * weightY;
      totalWeightX += weightX;
      totalWeightY += weightY;
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
    Logger.recordOutput(logPrefixStandard + "/FusePreviewFailed", false);

    // Return single fused estimate at the latest timestamp
    return List.of(
        new AprilCameraOutput(
            "fused",
            -1, // No single tag ID for fused
            new VisionPoseEstimation(fusedPose, fusedStdDevs, latestTimestamp)));
  }

  public Command createKillVisionCommand() {
    return new InstantCommand(() -> visionActive = false);
  }

  // ===== VISION PROCESSING METHODS (all computations replayable) =====

  /**
   * Process raw coprocessor data into a usable pose estimate. This method performs all RIO-side
   * computations (filtering, pose transformation, std dev calculation) so they can be replayed.
   */
  private VisionPoseEstimation processRawEstimate(
      AprilVisionInputs inputs, CameraConfiguration config, String cameraName) {

    double omegaRadPerSec =
        Math.abs(
            robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

    // Apply filtering
    if (!passesFilters(inputs, cameraName, omegaRadPerSec)) {
      return null;
    }

    // Transform pose for mechanism-mounted cameras
    Pose2d transformedPose = transformPoseForMechanism(inputs.rawCoprocessorPose, config);

    // Compute standard deviations
    OdometryStandardDevs stdDevs =
        computeStdDevs(inputs, config, cameraName, omegaRadPerSec, transformedPose);

    return new VisionPoseEstimation(
        transformedPose, stdDevs, inputs.coprocessorEstimationTimestamp);
  }

  /** Check if the estimate passes all filtering criteria. */
  private boolean passesFilters(
      AprilVisionInputs inputs, String cameraName, double omegaRadPerSec) {
    // Reject if too close (garbage data from being inside tag)
    boolean tooClose = inputs.avgTagDist < 0.56;

    // Rotation rate filtering - stricter for single tag
    boolean rotatingTooFast;
    if (inputs.tagCount == 1) {
      rotatingTooFast = omegaRadPerSec > MAX_OMEGA_FOR_SINGLE_TAG;
    } else {
      rotatingTooFast = omegaRadPerSec > MAX_OMEGA_FOR_ANY_TAG;
    }

    // Log rejection reasons
    Logger.recordOutput(logPrefixStandard + "/" + cameraName + "/omegaRadPerSec", omegaRadPerSec);
    Logger.recordOutput(logPrefixStandard + "/" + cameraName + "/rejectedTooClose", tooClose);
    Logger.recordOutput(
        logPrefixStandard + "/" + cameraName + "/rejectedRotation", rotatingTooFast);

    return !tooClose && !rotatingTooFast;
  }

  /** Transform the raw coprocessor pose to account for mechanism-mounted cameras (e.g., turret). */
  private Pose2d transformPoseForMechanism(Pose2d rawPose, CameraConfiguration config) {
    // Get mechanism yaw (e.g., turret rotation)
    Rotation2d mechanismYaw =
        Rotation2d.fromRadians(config.mechanismOrigin.get().getRotation().getZ());

    // LL returned pose has rotation = robotYaw + mechanismYaw, so subtract to get robot yaw
    Rotation2d robotYaw = rawPose.getRotation().minus(mechanismYaw);

    // Camera offset in robot frame - transform to field frame using robot yaw
    Pose3d cameraPosition = config.getCameraPosition();
    double offsetX = cameraPosition.getX();
    double offsetY = cameraPosition.getY();
    double cosYaw = robotYaw.getCos();
    double sinYaw = robotYaw.getSin();
    double fieldOffsetX = offsetX * cosYaw - offsetY * sinYaw;
    double fieldOffsetY = offsetX * sinYaw + offsetY * cosYaw;

    return new Pose2d(rawPose.getX() - fieldOffsetX, rawPose.getY() - fieldOffsetY, robotYaw);
  }

  /** Compute standard deviations for a vision estimate. */
  private OdometryStandardDevs computeStdDevs(
      AprilVisionInputs inputs,
      CameraConfiguration config,
      String cameraName,
      double omegaRadPerSec,
      Pose2d transformedPose) {

    // Get base std devs from coprocessor or fallback
    double baseStdDev = extractBaseStdDev(inputs, config);

    // Apply quality scaling based on tag area
    double quality = computeQualityScore(inputs.avgTagArea, inputs.tagCount);
    double qualityScaleFactor = 1.0 / quality;
    double scaledStdDev = baseStdDev * qualityScaleFactor;

    // Apply motion penalties
    double motionAdjustedStdDev = applyMotionPenalties(scaledStdDev, omegaRadPerSec);

    // Log pre-odom adjustment
    Logger.recordOutput(logPrefixStandard + "/" + cameraName + "/avgTagArea", inputs.avgTagArea);
    Logger.recordOutput(logPrefixStandard + "/" + cameraName + "/quality", quality);
    Logger.recordOutput(
        logPrefixStandard + "/" + cameraName + "/qualityScaleFactor", qualityScaleFactor);
    Logger.recordOutput(
        logPrefixStandard + "/" + cameraName + "/xyStdDevPreOdomAdjust", motionAdjustedStdDev);

    // Apply odometry divergence adjustment
    OdometryStandardDevs preAdjust =
        new OdometryStandardDevs(motionAdjustedStdDev, motionAdjustedStdDev, Double.MAX_VALUE);
    return adjustStdDevsWithOdomPose(
        preAdjust, inputs.coprocessorEstimationTimestamp, transformedPose);
  }

  /** Extract base standard deviation from Limelight array or use fallback calculation. */
  private double extractBaseStdDev(AprilVisionInputs inputs, CameraConfiguration config) {
    double[] stdDevArray = inputs.rawStdDevsArray;

    if (stdDevArray.length >= EXPECTED_STDDEV_ARRAY_LENGTH) {
      double xStdDev = stdDevArray[MEGATAG2_X_STDDEV_INDEX];
      double yStdDev = stdDevArray[MEGATAG2_Y_STDDEV_INDEX];

      if (xStdDev > 0 && yStdDev > 0) {
        return Math.max(xStdDev, yStdDev);
      }
    }

    // Fallback: calculate using distance and tag count
    double stdDevFactor = Math.pow(inputs.avgTagDist, 2) / Math.max(1, inputs.tagCount);
    return config.baselineTranslationalStdDev * stdDevFactor;
  }

  /** Compute quality score based on tag area (percentage of image). */
  private double computeQualityScore(double avgTagArea, int tagCount) {
    double quality;

    if (avgTagArea < TAG_AREA_REJECT_THRESHOLD) {
      quality = 0.05;
    } else if (avgTagArea < TAG_AREA_FAR_THRESHOLD) {
      double t =
          (avgTagArea - TAG_AREA_REJECT_THRESHOLD)
              / (TAG_AREA_FAR_THRESHOLD - TAG_AREA_REJECT_THRESHOLD);
      quality = 0.1 + (t * 0.2);
    } else if (avgTagArea < TAG_AREA_MEDIUM_THRESHOLD) {
      double t =
          (avgTagArea - TAG_AREA_FAR_THRESHOLD)
              / (TAG_AREA_MEDIUM_THRESHOLD - TAG_AREA_FAR_THRESHOLD);
      quality = 0.3 + (t * 0.3);
    } else if (avgTagArea < TAG_AREA_GOOD_THRESHOLD) {
      double t =
          (avgTagArea - TAG_AREA_MEDIUM_THRESHOLD)
              / (TAG_AREA_GOOD_THRESHOLD - TAG_AREA_MEDIUM_THRESHOLD);
      quality = 0.6 + (t * 0.3);
    } else {
      quality = 1.0;
    }

    // Multi-tag bonus
    if (tagCount > 1) {
      quality = Math.min(1.0, quality * 1.5);
    }

    return quality;
  }

  /**
   * Apply translation velocity penalty to std dev. During fast translation, vision measurements are
   * delayed by 100-240ms. At 1.5 m/s with 200ms latency, the robot moves 0.3m. Without sufficient
   * stddev scaling, the pose estimator over-trusts stale vision data, causing the estimate to lag
   * behind the actual robot position and "snap" when stopped.
   *
   * <p>Uses exponential scaling so vision stddev grows faster than linear with speed, ensuring
   * odometry dominates during fast motion while vision still provides drift correction at low
   * speeds.
   */
  private double applyMotionPenalties(double stdDev, double omegaRadPerSec) {
    // Get current translation velocity
    var chassisSpeeds = robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds();
    double translationalVelocity =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    // Exponential scaling: at 1 m/s -> 2x, at 1.5 m/s -> 2.8x, at 2 m/s -> 4x
    // This reduces vision weight from ~40% to ~7% at 1.5 m/s (with odom stddev of 0.3)
    double translationPenalty = Math.pow(2, translationalVelocity);
    stdDev *= translationPenalty;

    return stdDev;
  }

  /**
   * Adjust std devs based on divergence between vision and odometry estimates. Currently disabled -
   * the penalty could be preveting correcting from wrong initial positions and 254 doesn't use this
   * approach. Limelight's stddev + rotation rejection provide sufficient protection.
   */
  private OdometryStandardDevs adjustStdDevsWithOdomPose(
      OdometryStandardDevs unadjustedStdDevs, double timestampSeconds, Pose2d visionPose) {
    //    Pose2d odomPose = robotStateInstance.getFieldRobotPoseForTimestamp(timestampSeconds);

    //    if (odomPose == null || visionPose == null) {
    //      return unadjustedStdDevs;
    //    }

    //    double distMeters = odomPose.minus(visionPose).getTranslation().getNorm();
    //    double factor = 1 + (Math.pow(distMeters, 2) * 2);

    //    return new OdometryStandardDevs(
    //        unadjustedStdDevs.xStdDev() * factor,
    //        unadjustedStdDevs.yStdDev() * factor,
    //        unadjustedStdDevs.rotStdDev() * factor);
    return unadjustedStdDevs;
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
