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
import com.aembot.lib.tracing.Traced;
import com.aembot.lib.tracing.Tracer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Limelight4IOHardware implements AprilCameraIO {
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

  private AtomicReference<PoseEstimate> megatag2Estimate =
      new AtomicReference<>(new PoseEstimate());

  private AtomicReference<double[]> limelightStdDevs = new AtomicReference<>(new double[0]);

  protected final Consumer<NetworkTableEvent> heartbeatCallback = this::updateNtValuesCache;

  private double cachedRobotYaw = 0;

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

    LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraPosition.getX(),
        -cameraPosition.getY(),
        cameraPosition.getZ(),
        Units.radiansToDegrees(cameraPosition.getRotation().getX()),
        -Units.radiansToDegrees(cameraPosition.getRotation().getY()),
        Units.radiansToDegrees(cameraPosition.getRotation().getZ()));
  }

  /**
   * Updates the cached values from NetworkTables. Called asynchronously as {@link
   * #heartbeatCallback} for every limelight heartbeat
   */
  @Traced(category = "Vision")
  public void updateNtValuesCache(NetworkTableEvent event) {
    try (var ignored = Tracer.trace("LL.cache.getLatency." + cameraName, "Vision")) {
      latencyMs.set(
          LimelightHelpers.getLatency_Capture(cameraName)
              + LimelightHelpers.getLatency_Pipeline(cameraName));
    }

    try (var ignored = Tracer.trace("LL.cache.getBasicValues." + cameraName, "Vision")) {
      hasTag.set(LimelightHelpers.getTV(cameraName));
      tagID.set((int) LimelightHelpers.getFiducialID(cameraName));
    }

    try (var ignored = Tracer.trace("LL.cache.getMegatag2." + cameraName, "Vision")) {
      megatag2Estimate.set(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName));
    }

    try (var ignored = Tracer.trace("LL.cache.getStdDevs." + cameraName, "Vision")) {
      limelightStdDevs.set(LimelightExtras.getStandardDeviations(cameraName));
    }
  }

  @Override
  @Traced(category = "Vision")
  public void updateInputs(AprilVisionInputs inputs) {
    try (var ignored = Tracer.trace("LL.getAtomicValues." + cameraName, "Vision")) {
      inputs.latency = latencyMs.get();
      inputs.hasTag = hasTag.get();
      inputs.tagID = tagID.get();
    }

    try (var ignored = Tracer.trace("LL.getTagPosition." + cameraName, "Vision")) {
      if (inputs.hasTag && inputs.tagID >= 1 && inputs.tagID <= fieldConstants.getNumTags()) {
        inputs.tagPosition = fieldConstants.getAprilTagPose3d(inputs.tagID);
      } else {
        inputs.tagPosition = PositionUtil.NaN.POSE3D;
      }
    }

    VisionPoseEstimation coprocessorPoseEstimation;
    try (var ignored = Tracer.trace("LL.getMegatag2Estimate." + cameraName, "Vision")) {
      coprocessorPoseEstimation = getMegatag2Estimate();
    }

    inputs.coprocessorEstimationLatencyUncompensated =
        coprocessorPoseEstimation.latencyUncompensatedPose();
    inputs.coprocessorEstimationLatencyCompensated =
        coprocessorPoseEstimation.latencyCompensatedPose();
    inputs.coprocessorEstimationStdDevs = coprocessorPoseEstimation.stdDevs();
    inputs.coprocessorEstimationTimestamp = coprocessorPoseEstimation.timestampSeconds();

    try (var ignored = Tracer.trace("LL.logTemperature." + cameraName, "Vision")) {
      Logger.recordOutput(
          cameraName + "/tempCelsius", LimelightExtras.getCameraTemperature(cameraName));
    }
  }

  private void setRobotYawNetworkTables() {

    double robotYaw = robotStateInstance.getLatestFieldRobotPose().getRotation().getDegrees();
    double robotYawRate =
        Units.radiansToDegrees(
            robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
    double deltaYaw = robotYaw - cachedRobotYaw;

    // if (Math.abs(deltaYaw) > 0.25) {
    LimelightHelpers.SetRobotOrientation_NoFlush(cameraName, robotYaw, robotYawRate, 0, 0, 0, 0);
    cachedRobotYaw = robotYaw;
    // }
  }

  @Traced(category = "Vision")
  private VisionPoseEstimation getMegatag2Estimate() {

    VisionPoseEstimation poseEstimation;

    try (var ignored = Tracer.trace("LL.setRobotYawNT." + cameraName, "Vision")) {
      setRobotYawNetworkTables();
    }

    PoseEstimate estimate = megatag2Estimate.get();
    // Check that there actually is an estimate, and that we haven't processed it yet
    boolean garbageData = estimate.avgTagDist < 0.56; // TODO MAGIC NUMBER AAAAAA
    if (estimate.tagCount > 0
        && estimate.timestampSeconds != lastMegatag2Timestamp
        && !garbageData) {
      Pose2d latencyUncompensatedPose = estimate.pose;

      Pose2d latencyCompensatedPose;
      try (var ignored = Tracer.trace("LL.compensateLatency." + cameraName, "Vision")) {
        latencyCompensatedPose =
            compensateForEstimateLatency(
                estimate.pose,
                robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
                Timer.getFPGATimestamp() - (estimate.timestampSeconds));
      }

      lastMegatag2Timestamp = estimate.timestampSeconds;

      OdometryStandardDevs stdDevs;
      try (var ignored = Tracer.trace("LL.getStdDevs." + cameraName, "Vision")) {
        stdDevs = getStdDevs(estimate);
      }

      poseEstimation =
          new VisionPoseEstimation(
              latencyUncompensatedPose, latencyCompensatedPose, stdDevs, estimate.timestampSeconds);
    } else {
      poseEstimation = new VisionPoseEstimation(null, null, null, Double.NaN);
    }

    return poseEstimation;
  }

  /** Get standard deviations for the given pose estimate */
  protected OdometryStandardDevs getStdDevs(PoseEstimate estimate) {
    // Yoinked from 2481
    double stdDevFactor = Math.pow(estimate.avgTagDist, 2) / estimate.tagCount;

    double translationStddev = cameraConfiguration.baselineTranslationalStdDev * stdDevFactor;
    Double angularStddev = cameraConfiguration.baselineAngularStdDev * stdDevFactor;

    OdometryStandardDevs result;
    try (var ignored = Tracer.trace("LL.adjustStdDevs." + cameraName, "Vision")) {
      result =
          adjustStdDevsWithOdomPose(
              new OdometryStandardDevs(translationStddev, translationStddev, Double.MAX_VALUE),
              estimate.timestampSeconds,
              estimate.pose);
    }
    return result;
  }

  protected OdometryStandardDevs adjustStdDevsWithOdomPose(
      OdometryStandardDevs unadjustedStandardDevs,
      double timestampSeconds,
      Pose2d cameraEstimatedRobotPose) {
    Pose2d odomPose;
    try (var ignored = Tracer.trace("LL.getPoseForTimestamp." + cameraName, "Vision")) {
      odomPose = robotStateInstance.getFieldRobotPoseForTimestamp(timestampSeconds);
    }
    return adjustStdDevsWithOdomPose(unadjustedStandardDevs, odomPose, cameraEstimatedRobotPose);
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return cameraConfiguration;
  }

  @Override
  @Traced(category = "Vision")
  public void updateNetworkTablesForDisabled() {
    LimelightHelpers.SetThrottle(cameraName, this.cameraConfiguration.disabledThrottleValue);
    LimelightHelpers.SetIMUMode(cameraName, this.cameraConfiguration.disabledIMUMode);
  }

  @Override
  @Traced(category = "Vision")
  public void updateNetworkTablesForEnabled() {
    LimelightHelpers.SetThrottle(cameraName, this.cameraConfiguration.enabledThrottledValue);
    LimelightHelpers.SetIMUMode(cameraName, this.cameraConfiguration.enabledIMUMode);
  }
}
