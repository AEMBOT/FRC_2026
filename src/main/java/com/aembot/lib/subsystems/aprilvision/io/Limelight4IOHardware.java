package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.math.PositionUtil;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.LimelightExtras;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    megatag2Estimate.set(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName));

    limelightStdDevs.set(LimelightExtras.getStandardDeviations(cameraName));
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    // Update robot orientation for MegaTag2
    setRobotYawNetworkTables();

    inputs.latency = latencyMs.get();

    inputs.hasTag = hasTag.get();
    inputs.tagID = tagID.get();

    if (inputs.hasTag && inputs.tagID >= 1 && inputs.tagID <= fieldConstants.getNumTags()) {
      inputs.tagPosition = fieldConstants.getAprilTagPose3d(inputs.tagID);
    } else {
      inputs.tagPosition = PositionUtil.NaN.POSE3D;
    }

    // Populate RAW coprocessor data (no RIO-side processing)
    PoseEstimate estimate = megatag2Estimate.get();

    if (estimate.tagCount > 0 && estimate.timestampSeconds != lastMegatag2Timestamp) {
      inputs.rawCoprocessorPose = estimate.pose;
      inputs.avgTagDist = estimate.avgTagDist;
      inputs.avgTagArea = estimate.avgTagArea;
      inputs.tagCount = estimate.tagCount;
      inputs.coprocessorEstimationTimestamp = estimate.timestampSeconds;
      inputs.rawStdDevsArray = limelightStdDevs.get();

      lastMegatag2Timestamp = estimate.timestampSeconds;
    } else {
      inputs.rawCoprocessorPose = null;
      inputs.tagCount = 0;
    }

    Logger.recordOutput(
        cameraName + "/tempCelsius", LimelightExtras.getCameraTemperature(cameraName));
  }

  private void setRobotYawNetworkTables() {
    double robotYaw = robotStateInstance.getLatestFieldRobotPose().getRotation().getDegrees();

    // Add mechanism origin yaw (e.g., turret rotation) so the LL knows its actual field orientation
    double mechanismYawDegrees =
        Units.radiansToDegrees(cameraConfiguration.mechanismOrigin.get().getRotation().getZ());

    // For mechanism-mounted cameras, set yaw rate to 0 since we don't have mechanism velocity
    double yawRate =
        mechanismYawDegrees == 0
            ? Units.radiansToDegrees(
                robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds()
                    .omegaRadiansPerSecond)
            : 0;

    LimelightHelpers.SetRobotOrientation_NoFlush(
        cameraName, robotYaw + mechanismYawDegrees, yawRate, 0, 0, 0, 0);
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
