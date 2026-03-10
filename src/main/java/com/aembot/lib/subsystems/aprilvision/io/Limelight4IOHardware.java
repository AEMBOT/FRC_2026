package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.math.PositionUtil;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.LimelightExtras;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.PoseEstimate;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

public class Limelight4IOHardware implements AprilCameraIO {
  // MegaTag2 stddev indices in Limelight's 12-element stddevs array
  private static final int MEGATAG2_X_STDDEV_INDEX = 6;
  private static final int MEGATAG2_Y_STDDEV_INDEX = 7;
  private static final int EXPECTED_STDDEV_ARRAY_LENGTH = 12;

  // Fallback stddev when Limelight doesn't provide valid data
  private static final double FALLBACK_TRANSLATION_STDDEV = 0.5;

  // Filtering thresholds for MegaTag2 best practices
  /** Max rotation rate (rad/s) before rejecting single-tag estimates entirely */
  private static final double MAX_OMEGA_FOR_SINGLE_TAG = Units.degreesToRadians(150);

  /** Max rotation rate (rad/s) before rejecting all estimates entirely */
  private static final double MAX_OMEGA_FOR_ANY_TAG = Units.degreesToRadians(360);

  // Tag area thresholds (percentage of image) for quality scoring
  /** Below this area, tag is too far / detection unreliable - reject or heavy penalty */
  private static final double TAG_AREA_REJECT_THRESHOLD = 0.05;

  /** Below this area, tag is far but usable - reduced trust */
  private static final double TAG_AREA_FAR_THRESHOLD = 0.1;

  /** Below this area, tag is at medium range - normal trust */
  private static final double TAG_AREA_MEDIUM_THRESHOLD = 1.0;

  /** Below this area, tag is at good range - high trust */
  private static final double TAG_AREA_GOOD_THRESHOLD = 5.0;

  // Above GOOD_THRESHOLD = very close, highest trust

  protected final CameraConfiguration cameraConfiguration;
  protected final YearFieldConstantable fieldConstants;

  /**
   * The name of the camera as it appears on NetworkTables and as its hostname. Ex:
   * "limelight-FrontLeft" will map to "limelight-FrontLeft" on NT4 & "limelight-frontleft.local"
   * over mDNS
   */
  public final String cameraName;

  protected final NetworkTable networkTable;

  /**
   * double NT entry indicating the heartbeat of the limelight. "hb" on network table. Resets at
   * two-billion
   */
  protected final NetworkTableEntry heartbeatEntry;

  protected final List<Point> tagCorners =
      List.of(new Point(), new Point(), new Point(), new Point());

  protected final RobotState robotStateInstance;

  /**
   * The timestamp of the last megatag2 pose estimation. Two estimates will not be used from the
   * same timestamp (or, more importantly, the same estimate from the same timestamp.)
   */
  private double lastMegatag2Timestamp = Double.NaN;

  /* ---- ASYNCHRONOUSLY UPDATED FIELDS ---- */
  private AtomicReference<Double> latencyMs = new AtomicReference<>(0.0);

  private AtomicBoolean hasTag = new AtomicBoolean(false);

  private AtomicInteger tagID = new AtomicInteger(-1);

  private AtomicReference<Rotation2d> horizontalAngleToTagRadians =
      new AtomicReference<>(PositionUtil.NaN.ROTATION2D);

  private AtomicReference<double[]> tagCornerPositionsRaw = new AtomicReference<>(new double[0]);

  private AtomicReference<PoseEstimate> megatag2Estimate =
      new AtomicReference<>(new PoseEstimate());

  private AtomicReference<double[]> limelightStdDevs = new AtomicReference<>(new double[0]);

  protected final Consumer<NetworkTableEvent> heartbeatCallback = this::updateNtValuesCache;

  protected Transform2d horizontalCameraOffset;

  public Limelight4IOHardware(
      CameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance) {
    this.cameraConfiguration = config;
    this.cameraName = "limelight-" + config.cameraName;
    this.fieldConstants = fieldConstants;
    this.robotStateInstance = robotStateInstance;

    this.networkTable = NetworkTableInstance.getDefault().getTable(cameraName);
    this.heartbeatEntry = networkTable.getEntry("hb");

    NetworkTableInstance.getDefault()
        .addListener(
            heartbeatEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), heartbeatCallback);

    Pose3d cameraPosition = cameraConfiguration.getCameraPosition();

    horizontalCameraOffset =
        new Transform2d(-cameraPosition.getX(), -cameraPosition.getY(), Rotation2d.kZero);

    LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        0,
        0,
        cameraPosition.getZ(),
        Units.radiansToDegrees(cameraPosition.getRotation().getX()),
        -Units.radiansToDegrees(cameraPosition.getRotation().getY()),
        Units.radiansToDegrees(cameraPosition.getRotation().getZ()));
  }

  /**
   * Updates the cached values from NetworkTables. Called asynchronously as {@link
   * #heartbeatCallback} for every limelight heartbeat
   */
  public void updateNtValuesCache(NetworkTableEvent event) {
    latencyMs.set(
        LimelightHelpers.getLatency_Capture(cameraName)
            + LimelightHelpers.getLatency_Pipeline(cameraName));

    hasTag.set(LimelightHelpers.getTV(cameraName));
    tagID.set((int) LimelightHelpers.getFiducialID(cameraName));
    horizontalAngleToTagRadians.set(Rotation2d.fromDegrees(LimelightHelpers.getTX(cameraName)));
    tagCornerPositionsRaw.set(LimelightHelpers.getCornerCoordinates(cameraName));

    megatag2Estimate.set(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName));

    limelightStdDevs.set(LimelightExtras.getStandardDeviations(cameraName));
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    inputs.latency = latencyMs.get();

    inputs.hasTag = hasTag.get();
    inputs.tagID = tagID.get();
    inputs.horizontalAngleToTag = horizontalAngleToTagRadians.get();

    updateCornerPositions();
    inputs.tagCornerPositions = this.tagCorners;

    if (inputs.hasTag && inputs.tagID >= 1 && inputs.tagID <= fieldConstants.getNumTags()) {
      inputs.tagPosition = fieldConstants.getAprilTagPose3d(inputs.tagID);
    } else {
      inputs.tagPosition = PositionUtil.NaN.POSE3D;
    }

    VisionPoseEstimation coprocessorPoseEstimation = getMegatag2Estimate();

    inputs.coprocessorEstimationLatencyUncompensated =
        coprocessorPoseEstimation.latencyUncompensatedPose();
    inputs.coprocessorEstimationLatencyCompensated =
        coprocessorPoseEstimation.latencyCompensatedPose();
    inputs.coprocessorEstimationStdDevs = coprocessorPoseEstimation.stdDevs();
    inputs.coprocessorEstimationTimestamp = coprocessorPoseEstimation.timestampSeconds();

    Logger.recordOutput(
        cameraName + "/tempCelsius", LimelightExtras.getCameraTemperature(cameraName));
  }

  private void setRobotYawNetworkTables() {
    double robotYaw = robotStateInstance.getLatestFieldRobotPose().getRotation().getDegrees();
    double robotYawRate =
        Units.radiansToDegrees(
            robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation_NoFlush(cameraName, robotYaw, robotYawRate, 0, 0, 0, 0);
  }

  private VisionPoseEstimation getMegatag2Estimate() {
    setRobotYawNetworkTables();

    PoseEstimate estimate = megatag2Estimate.get();

    // Early exit if no valid estimate or already processed
    if (estimate.tagCount <= 0 || estimate.timestampSeconds == lastMegatag2Timestamp) {
      return new VisionPoseEstimation(null, null, null, Double.NaN);
    }

    // Get current rotation rate for filtering
    double omegaRadPerSec =
        Math.abs(
            robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

    // === FILTERING CHECKS ===

    // 1. Reject if too close (garbage data from being inside tag)
    boolean tooClose = estimate.avgTagDist < 0.56;

    // 2. Rotation rate filtering - stricter for single tag
    boolean rotatingTooFast;
    if (estimate.tagCount == 1) {
      rotatingTooFast = omegaRadPerSec > MAX_OMEGA_FOR_SINGLE_TAG;
    } else {
      rotatingTooFast = omegaRadPerSec > MAX_OMEGA_FOR_ANY_TAG;
    }

    // Log rejection reasons for debugging
    Logger.recordOutput(cameraName + "/Vision/omegaRadPerSec", omegaRadPerSec);
    Logger.recordOutput(cameraName + "/Vision/rejectedTooClose", tooClose);
    Logger.recordOutput(cameraName + "/Vision/rejectedRotation", rotatingTooFast);

    if (tooClose || rotatingTooFast) {
      return new VisionPoseEstimation(null, null, null, Double.NaN);
    }

    // === POSE PROCESSING ===

    Pose2d latencyUncompensatedPose = estimate.pose.plus(horizontalCameraOffset);
    Pose2d latencyCompensatedPose =
        compensateForEstimateLatency(
            latencyUncompensatedPose,
            robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
            Timer.getFPGATimestamp() - estimate.timestampSeconds);

    lastMegatag2Timestamp = estimate.timestampSeconds;

    return new VisionPoseEstimation(
        latencyUncompensatedPose,
        latencyCompensatedPose,
        getStdDevs(estimate),
        estimate.timestampSeconds);
  }

  /**
   * Convert the Limelight-provided double[] of corner coordinates into a list of Vector2s
   * representing the corner positions in pixels
   */
  private void updateCornerPositions() {
    double[] cornerPositions = tagCornerPositionsRaw.get();

    if (cornerPositions.length >= 8) {
      // 4 iterations bcuz we process 2 at a time
      for (int i = 0; i < 4; i++) {
        tagCorners.get(i).x = cornerPositions[i * 2];
        tagCorners.get(i).y = cornerPositions[i * 2 + 1];
      }
    }
  }

  /**
   * Get standard deviations for the given pose estimate using Limelight-provided stddevs with
   * quality scaling. Implements best practices from top FRC teams for MegaTag2.
   */
  protected OdometryStandardDevs getStdDevs(PoseEstimate estimate) {
    double[] stdDevArray = limelightStdDevs.get();

    double xStdDev;
    double yStdDev;

    // Use Limelight-provided stddevs if available, otherwise fall back to calculated
    if (stdDevArray.length >= EXPECTED_STDDEV_ARRAY_LENGTH) {
      xStdDev = stdDevArray[MEGATAG2_X_STDDEV_INDEX];
      yStdDev = stdDevArray[MEGATAG2_Y_STDDEV_INDEX];

      // Sanity check - if Limelight returns 0 or negative, use fallback
      if (xStdDev <= 0 || yStdDev <= 0) {
        xStdDev = FALLBACK_TRANSLATION_STDDEV;
        yStdDev = FALLBACK_TRANSLATION_STDDEV;
      }
    } else {
      // Fallback: calculate stddev using distance and tag count (original method)
      double stdDevFactor = Math.pow(estimate.avgTagDist, 2) / estimate.tagCount;
      xStdDev = cameraConfiguration.baselineTranslationalStdDev * stdDevFactor;
      yStdDev = xStdDev;
    }

    // Calculate quality score based on tag area ranges
    // avgTagArea is percentage of image, typical values 0.05% - 10%+
    double quality;
    double avgArea = estimate.avgTagArea;

    if (avgArea < TAG_AREA_REJECT_THRESHOLD) {
      // Too far / unreliable detection - very low quality
      quality = 0.05;
    } else if (avgArea < TAG_AREA_FAR_THRESHOLD) {
      // Far but usable (0.05% - 0.1%) - low quality, interpolate 0.1 to 0.3
      double t = (avgArea - TAG_AREA_REJECT_THRESHOLD) / (TAG_AREA_FAR_THRESHOLD - TAG_AREA_REJECT_THRESHOLD);
      quality = 0.1 + (t * 0.2);
    } else if (avgArea < TAG_AREA_MEDIUM_THRESHOLD) {
      // Medium range (0.1% - 1%) - moderate quality, interpolate 0.3 to 0.6
      double t = (avgArea - TAG_AREA_FAR_THRESHOLD) / (TAG_AREA_MEDIUM_THRESHOLD - TAG_AREA_FAR_THRESHOLD);
      quality = 0.3 + (t * 0.3);
    } else if (avgArea < TAG_AREA_GOOD_THRESHOLD) {
      // Good range (1% - 5%) - high quality, interpolate 0.6 to 0.9
      double t = (avgArea - TAG_AREA_MEDIUM_THRESHOLD) / (TAG_AREA_GOOD_THRESHOLD - TAG_AREA_MEDIUM_THRESHOLD);
      quality = 0.6 + (t * 0.3);
    } else {
      // Very close (>5%) - excellent quality
      quality = 1.0;
    }

    // Multi-tag bonus: boost quality when multiple tags visible
    if (estimate.tagCount > 1) {
      // Scale up quality, but cap at 1.0
      quality = Math.min(1.0, quality * 1.5);
    }

    // Log area for debugging
    Logger.recordOutput(cameraName + "/Vision/avgTagArea", avgArea);

    // Scale stddevs by inverse quality (lower quality = higher stddev = less trust)
    double qualityScaleFactor = 1.0 / quality;
    xStdDev *= qualityScaleFactor;
    yStdDev *= qualityScaleFactor;

    // Get chassis speeds for motion penalties
    var chassisSpeeds = robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds();

    // Apply rotation rate penalty - MegaTag2 becomes less reliable when rotating
    double omegaRadPerSec = Math.abs(chassisSpeeds.omegaRadiansPerSecond);
    // Linear penalty: at 1 rad/s (~57°/s), stddev is doubled
    double rotationPenalty = 1.0 + omegaRadPerSec;
    xStdDev *= rotationPenalty;
    yStdDev *= rotationPenalty;

    // Apply translational velocity penalty - motion blur degrades vision
    double translationalVelocity =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    // Gentle penalty: at 2 m/s, stddev increases by 50%
    double translationPenalty = 1.0 + (translationalVelocity * 0.25);
    xStdDev *= translationPenalty;
    yStdDev *= translationPenalty;

    // Use the larger of the two for a conservative estimate
    double xyStdDev = Math.max(xStdDev, yStdDev);

    // Log for debugging
    Logger.recordOutput(cameraName + "/Vision/quality", quality);
    Logger.recordOutput(cameraName + "/Vision/qualityScaleFactor", qualityScaleFactor);
    Logger.recordOutput(cameraName + "/Vision/rotationPenalty", rotationPenalty);
    Logger.recordOutput(cameraName + "/Vision/translationPenalty", translationPenalty);
    Logger.recordOutput(cameraName + "/Vision/xyStdDevPreOdomAdjust", xyStdDev);

    // Apply final adjustment based on odometry vs vision pose divergence
    return adjustStdDevsWithOdomPose(
        new OdometryStandardDevs(xyStdDev, xyStdDev, Double.MAX_VALUE),
        estimate.timestampSeconds,
        estimate.pose);
  }

  protected OdometryStandardDevs adjustStdDevsWithOdomPose(
      OdometryStandardDevs unadjustedStandardDevs,
      double timestampSeconds,
      Pose2d cameraEstimatedRobotPose) {
    return adjustStdDevsWithOdomPose(
        unadjustedStandardDevs,
        robotStateInstance.getFieldRobotPoseForTimestamp(timestampSeconds),
        cameraEstimatedRobotPose);
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return cameraConfiguration;
  }

  @Override
  public void updateNetworkTablesForDisabled() {
    LimelightHelpers.SetThrottle(cameraName, this.cameraConfiguration.disabledThrottleValue);
    LimelightHelpers.SetIMUMode(cameraName, this.cameraConfiguration.disabledIMUMode);
  }

  @Override
  public void updateNetworkTablesForEnabled() {
    LimelightHelpers.SetThrottle(cameraName, this.cameraConfiguration.enabledThrottledValue);
    LimelightHelpers.SetIMUMode(cameraName, this.cameraConfiguration.enabledIMUMode);
  }
}
