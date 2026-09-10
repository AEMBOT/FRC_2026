package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.LimelightExtras;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.PoseEstimate;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

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

  protected final RobotState robotStateInstance;

  /**
   * The timestamp of the last megatag2 pose estimation. Two estimates will not be used from the
   * same timestamp (or, more importantly, the same estimate from the same timestamp.)
   */
  private double lastMegatag2Timestamp = Double.NaN;

  /* ---- NT SUBSCRIBER ---- */
  private final DoubleArraySubscriber botposeSubscriber;

  /**
   * A parsed botpose frame plus the timing and validity data that goes with it. Bundled into a
   * single immutable object so the main thread cannot read a pose from one frame alongside the
   * timing of another.
   *
   * @param estimate the parsed MegaTag2 estimate, with per-tag data in {@code rawFiducials}
   * @param ntTimestampMicros raw NetworkTables timestamp of the frame, before latency subtraction
   * @param receivedFpgaTimestamp FPGA time at which the frame was received by the RIO
   * @param arrayLengthValid whether the array was long enough to contain per-tag data
   */
  private record CachedFrame(
      PoseEstimate estimate,
      long ntTimestampMicros,
      double receivedFpgaTimestamp,
      boolean arrayLengthValid) {}

  private static final CachedFrame EMPTY_FRAME = new CachedFrame(new PoseEstimate(), 0, 0, true);

  /** Number of values the Limelight appends per detected tag after the 11 header values. */
  private static final int VALS_PER_FIDUCIAL = 7;

  private static final int BOTPOSE_HEADER_VALS = 11;

  /* ---- ASYNCHRONOUSLY UPDATED FIELDS ---- */
  private AtomicReference<CachedFrame> megatag2Estimate = new AtomicReference<>(EMPTY_FRAME);

  private AtomicReference<double[]> limelightStdDevs = new AtomicReference<>(new double[0]);

  protected final Consumer<NetworkTableEvent> poseUpdateCallback = this::updateNtValuesCache;

  public Limelight4IOHardware(
      CameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance) {
    this.cameraConfiguration = config;
    this.cameraName = "limelight-" + config.cameraName;
    this.fieldConstants = fieldConstants;
    this.robotStateInstance = robotStateInstance;

    this.networkTable = NetworkTableInstance.getDefault().getTable(cameraName);

    // Subscribe to MegaTag2 pose data - callback fires when new pose arrives
    this.botposeSubscriber =
        networkTable
            .getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(new double[] {}, PubSubOption.sendAll(true));

    NetworkTableInstance.getDefault()
        .addListener(
            botposeSubscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), poseUpdateCallback);

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
   * Updates the cached values from NetworkTables. Called asynchronously when new MegaTag2 pose data
   * arrives on the botpose_orb_wpiblue topic.
   */
  private void updateNtValuesCache(NetworkTableEvent event) {
    // Get data directly from subscriber - no additional NT read needed
    TimestampedDoubleArray tsValue = botposeSubscriber.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) {
      megatag2Estimate.set(EMPTY_FRAME);
      return;
    }

    // Parse pose data from array (same format as LimelightHelpers)
    Pose2d pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);

    // Convert server timestamp from microseconds to seconds and adjust for latency
    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    // Per-tag data is already in this array; parse it rather than paying for another NT read.
    // A short array means the frame is malformed or truncated, so the per-tag values cannot be
    // trusted -- fall back to no fiducials and flag it, but keep using the header values so pose
    // behaviour is unchanged.
    // Exact match, not >=: a longer-than-expected array means the protocol has drifted (e.g. extra
    // header values), and the 11 + 7i indexing below would then read misaligned data rather than
    // failing. Upstream LimelightHelpers checks the same way.
    boolean arrayLengthValid =
        poseArray.length == BOTPOSE_HEADER_VALS + (VALS_PER_FIDUCIAL * tagCount);
    RawFiducial[] rawFiducials =
        arrayLengthValid ? extractFiducials(poseArray, tagCount) : new RawFiducial[0];

    PoseEstimate poseEstimate =
        new PoseEstimate(
            pose,
            adjustedTimestamp,
            latency,
            tagCount,
            tagSpan,
            tagDist,
            tagArea,
            rawFiducials,
            true);

    megatag2Estimate.set(
        new CachedFrame(poseEstimate, timestamp, Timer.getFPGATimestamp(), arrayLengthValid));
    // Read stddevs synchronously here to ensure they match the pose frame
    limelightStdDevs.set(LimelightExtras.getStandardDeviations(cameraName));
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      return new Pose2d();
    }
    return new Pose2d(
        new Translation2d(inData[0], inData[1]), new Rotation2d(Units.degreesToRadians(inData[5])));
  }

  /**
   * Pull the per-tag block out of a botpose array. The caller must have already checked that the
   * array is long enough.
   */
  private static RawFiducial[] extractFiducials(double[] inData, int tagCount) {
    RawFiducial[] fiducials = new RawFiducial[tagCount];

    for (int i = 0; i < tagCount; i++) {
      int baseIndex = BOTPOSE_HEADER_VALS + (i * VALS_PER_FIDUCIAL);
      fiducials[i] =
          new RawFiducial(
              (int) inData[baseIndex],
              inData[baseIndex + 1],
              inData[baseIndex + 2],
              inData[baseIndex + 3],
              inData[baseIndex + 4],
              inData[baseIndex + 5],
              inData[baseIndex + 6]);
    }

    return fiducials;
  }

  private static double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    // Update robot orientation for MegaTag2
    setRobotYawNetworkTables();

    // Read the cached frame once: two separate reads could straddle a callback and mix data from
    // different frames.
    CachedFrame frame = megatag2Estimate.get();
    PoseEstimate estimate = frame.estimate();

    inputs.latency = estimate.latency;

    inputs.hasTag = estimate.tagCount > 0;

    if (estimate.tagCount > 0 && estimate.timestampSeconds != lastMegatag2Timestamp) {
      inputs.rawCoprocessorPose = estimate.pose;
      inputs.avgTagDist = estimate.avgTagDist;
      inputs.avgTagArea = estimate.avgTagArea;
      inputs.tagCount = estimate.tagCount;
      inputs.coprocessorEstimationTimestamp = estimate.timestampSeconds;
      inputs.rawStdDevsArray = limelightStdDevs.get();

      inputs.rawNtTimestampMicros = frame.ntTimestampMicros();
      inputs.frameReceivedFpgaTimestamp = frame.receivedFpgaTimestamp();
      inputs.rawArrayLengthValid = frame.arrayLengthValid();
      copyFiducialsToInputs(estimate.rawFiducials, inputs);

      lastMegatag2Timestamp = estimate.timestampSeconds;
    } else {
      inputs.rawCoprocessorPose = null;
      inputs.tagCount = 0;

      // Clear per-tag data too, so a camera that has stopped seeing tags does not keep reporting
      // the tags from its last good frame.
      clearFiducialInputs(inputs);
    }

    AEMLogger.recordOutput(
        cameraName + "/tempCelsius", LimelightExtras.getCameraTemperature(cameraName));
  }

  private static void copyFiducialsToInputs(RawFiducial[] fiducials, AprilVisionInputs inputs) {
    if (inputs.tagIDs.length != fiducials.length) {
      inputs.tagIDs = new int[fiducials.length];
      inputs.tagDistancesToCamera = new double[fiducials.length];
      inputs.tagAmbiguities = new double[fiducials.length];
      inputs.tagAreas = new double[fiducials.length];
    }

    for (int i = 0; i < fiducials.length; i++) {
      inputs.tagIDs[i] = fiducials[i].id;
      inputs.tagDistancesToCamera[i] = fiducials[i].distToCamera;
      inputs.tagAmbiguities[i] = fiducials[i].ambiguity;
      inputs.tagAreas[i] = fiducials[i].ta;
    }
  }

  private static void clearFiducialInputs(AprilVisionInputs inputs) {
    if (inputs.tagIDs.length != 0) {
      inputs.tagIDs = new int[0];
      inputs.tagDistancesToCamera = new double[0];
      inputs.tagAmbiguities = new double[0];
      inputs.tagAreas = new double[0];
    }
  }

  private void setRobotYawNetworkTables() {
    double robotYaw = robotStateInstance.getLatestFieldRobotPose().getRotation().getDegrees();

    // Add mechanism origin rotation (e.g., turret) so the LL knows its actual field orientation
    Rotation3d mechanismRotation = cameraConfiguration.mechanismOrigin.get().getRotation();
    double mechanismYawDegrees = Units.radiansToDegrees(mechanismRotation.getZ());
    double mechanismPitchDegrees = Units.radiansToDegrees(mechanismRotation.getY());
    double mechanismRollDegrees = Units.radiansToDegrees(mechanismRotation.getX());

    // For mechanism-mounted cameras, set yaw rate to 0 since we don't have mechanism velocity
    double yawRate =
        mechanismYawDegrees == 0
            ? Units.radiansToDegrees(
                robotStateInstance.getLatestMeasuredFieldRelativeChassisSpeeds()
                    .omegaRadiansPerSecond)
            : 0;

    LimelightHelpers.SetRobotOrientation_NoFlush(
        cameraName,
        robotYaw + mechanismYawDegrees,
        yawRate,
        mechanismPitchDegrees,
        0,
        mechanismRollDegrees,
        0);
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
