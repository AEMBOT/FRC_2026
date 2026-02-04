package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.util.CameraCalibration;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.PoseEstimate;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

public class Limelight4IOHardware extends LimelightIO {
  /** bool indicating whether we have a tag targetted. "tv" on network table */
  protected boolean validTag;

  /**
   * double indicating horizontal offset in degrees from crosshair to target. "tx" on network table
   */
  protected double xOffset;

  /** int NT entry indicating id of the targeted apriltag. "tid" on network table */
  // protected final NetworkTableEntry tagIDEntry;

  protected int tagID;

  /**
   * double[] indicating corner coordinates in pixels [x0,y0,x1,y1......]. "tcornxy" on network
   * table.
   */
  protected double[] tagCornerPositions;

  /**
   * double indicating capture pipeline latency (ms). Time between the end of the exposure of the
   * middle row of the sensor to the beginning of the tracking pipeline. Sum with {@link
   * #pipelineLatency} to get total latency. "cl" on network table.
   */
  protected double captureLatency;

  /**
   * double indicating The pipeline's latency contribution (ms). Sum with {@link #captureLatency} to
   * get total latency. "tl" on network table (for some reason).
   */
  protected double pipelineLatency;

  protected final CameraConfiguration cameraConfiguration;
  protected final YearFieldConstantable fieldConstants;

  /**
   * The name of the camera as it appears on NetworkTables and as its hostname. Ex:
   * "limelight-FrontLeft" will map to "limelight-FrontLeft" on NT4 & "limelight-frontleft.local"
   * over mDNS
   */
  public final String cameraName;

  protected final List<Point> tagCorners =
      List.of(new Point(), new Point(), new Point(), new Point());

  protected final RobotState robotStateInstance;

  /**
   * The timestamp of the last megatag2 pose estimation. Two estimates will not be used from the
   * same timestamp (or, more importantly, the same estimate from the same timestamp.)
   */
  private double lastMegatag2Timestamp = Double.NaN;

  public Limelight4IOHardware(
      CameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance) {
    this.cameraConfiguration = config;
    this.cameraName = "limelight-" + config.cameraName;
    this.fieldConstants = fieldConstants;
    this.robotStateInstance = robotStateInstance;
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    // Camera calib should only be updated once
    if (inputs.cameraCalibration == null) {
      inputs.cameraCalibration = getCalibration();
    }

    inputs.latency =
        LimelightHelpers.getLatency_Capture(cameraName)
            + LimelightHelpers.getLatency_Pipeline(cameraName);

    inputs.hasTag =
        LimelightHelpers.getFiducialID(cameraName) != 0; // best method I could figure out
    inputs.tagID = (int) LimelightHelpers.getFiducialID(cameraName);
    inputs.horizontalAngleToTag = Rotation2d.fromDegrees(LimelightHelpers.getTX(cameraName));

    updateCornerPositions();
    inputs.tagCornerPositions = this.tagCorners;

    estimatePoseAndPublish(inputs);

    VisionPoseEstimation coprocessorPoseEstimation = getMegatag2Estimate();

    inputs.coprocessorEstimationLatencyUncompensated =
        coprocessorPoseEstimation.latencyUncompensatedPose();
    inputs.coprocessorEstimationLatencyCompensated =
        coprocessorPoseEstimation.latencyCompensatedPose();
    inputs.coprocessorEstimationStdDevs = coprocessorPoseEstimation.stdDevs();
    inputs.coprocessorEstimationTimestamp = coprocessorPoseEstimation.timestampSeconds();
  }

  /**
   * Using the given {@link AprilVisionInputs}, estimate the pose of the robot and publish data to
   * the inputs. The published inputs are:
   *
   * <ul>
   *   <li>{@link AprilVisionInputs#tagHeightPixels}
   *   <li>{@link AprilVisionInputs#tagHeightAngle}
   *   <li>{@link AprilVisionInputs#tagDistanceMeters}
   *   <li>{@link AprilVisionInputs#robotPoseEstimationLatencyUncompensated}
   *   <li>{@link AprilVisionInputs#robotPoseEstimationLatencyCompensated}
   *
   * @param inputs The {@link AprilVisionInputs} passed to {@link #updateInputs(AprilVisionInputs)}
   */
  private void estimatePoseAndPublish(AprilVisionInputs inputs) {
    // Initially set inputs to defaults
    inputs.tagHeightPixels = -1;
    inputs.tagHeightAngle = Rotation2d.kZero;
    inputs.tagDistanceMeters = -1;
    inputs.robotPoseEstimationLatencyUncompensated = null;
    inputs.robotPoseEstimationLatencyCompensated = null;
    inputs.tagPosition = null;
    inputs.simpleEstimationTimestamp = Double.NaN;
    if (inputs.hasTag) {
      inputs.tagHeightPixels = computeTagHeightInPixels(inputs.tagCornerPositions);
      inputs.tagHeightAngle = computeTagHeightInRotations(inputs.tagHeightPixels);

      // If tag id valid
      if (inputs.tagID >= 1 && inputs.tagID <= fieldConstants.getNumTags()) {
        inputs.tagPosition = fieldConstants.getAprilTagPose3d(inputs.tagID);

        inputs.tagDistanceMeters =
            computeDistanceToTagMeters(inputs.tagHeightAngle, inputs.tagPosition.getZ())
                * getConfiguration().cameraDistanceScalar;

        Rotation2d robotRotation = robotStateInstance.getLatestFieldRobotPose().getRotation();

        Translation2d robotToTagTranslation =
            computeRobotToTag(inputs.horizontalAngleToTag, inputs.tagDistanceMeters);

        VisionPoseEstimation poseEstimation =
            computeRobotPose(
                inputs.tagPosition.toPose2d(),
                robotRotation,
                robotToTagTranslation,
                robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
                inputs.latency);

        inputs.robotPoseEstimationLatencyUncompensated = poseEstimation.latencyUncompensatedPose();
        inputs.robotPoseEstimationLatencyCompensated = poseEstimation.latencyCompensatedPose();
        inputs.simpleEstimationStdDevs = poseEstimation.stdDevs();
        inputs.simpleEstimationTimestamp = poseEstimation.timestampSeconds();
      }
    }
  }

  private VisionPoseEstimation getMegatag2Estimate() {
    double robotYaw = robotStateInstance.getLatestFieldRobotPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(cameraName, robotYaw, 0, 0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    // Check that there actually is an estimate, and that we haven't processed it yet
    if (estimate.tagCount > 0 && estimate.timestampSeconds != lastMegatag2Timestamp) {
      Pose2d latencyUncompensatedPose = estimate.pose;
      Pose2d latencyCompensatedPose =
          compensateForEstimateLatency(
              estimate.pose,
              robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
              Timer.getFPGATimestamp() - (estimate.timestampSeconds - estimate.latency));

      // Yoinked from 2481
      double stdDevFactor = Math.pow(estimate.avgTagDist, 2) / estimate.tagCount;

      double translationStddev = cameraConfiguration.baselineTranslationalStdDev * stdDevFactor;
      Double angularStddev = cameraConfiguration.baselineAngularStdDev * stdDevFactor;

      return new VisionPoseEstimation(
          latencyUncompensatedPose,
          latencyCompensatedPose,
          new OdometryStandardDevs(translationStddev, translationStddev, angularStddev),
          estimate.timestampSeconds);
    } else {
      return new VisionPoseEstimation(null, null, null, Double.NaN);
    }
  }

  /**
   * Convert the Limelight-provided double[] of corner coordinates into a list of Vector2s
   * representing the corner positions in pixels
   */
  private void updateCornerPositions() {
    double[] cornerPositions = LimelightHelpers.getCornerCoordinates(cameraName);
    Logger.recordOutput(cameraConfiguration.cameraName + "corners", cornerPositions);

    if (cornerPositions.length >= 8) {
      // 4 iterations bcuz we process 2 at a time
      for (int i = 0; i < 4; i++) {
        tagCorners.get(i).x = cornerPositions[i * 2];
        tagCorners.get(i).y = cornerPositions[i * 2 + 1];
      }
    }
  }

  protected CameraCalibration getCalibration() {
    // TODO stub
    return null;
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return cameraConfiguration;
  }
}
