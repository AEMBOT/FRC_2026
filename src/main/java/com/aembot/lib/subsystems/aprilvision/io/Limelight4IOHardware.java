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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import org.opencv.core.Point;

public class Limelight4IOHardware implements AprilCameraIO {
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
    inputs.latency =
        LimelightHelpers.getLatency_Capture(cameraName)
            + LimelightHelpers.getLatency_Pipeline(cameraName);

    inputs.hasTag = LimelightHelpers.getTV(cameraName);
    inputs.tagID = (int) LimelightHelpers.getFiducialID(cameraName);
    inputs.horizontalAngleToTag = Rotation2d.fromDegrees(LimelightHelpers.getTX(cameraName));

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
  }

  private VisionPoseEstimation getMegatag2Estimate() {
    LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        cameraConfiguration.getCameraPosition().getX(),
        cameraConfiguration.getCameraPosition().getY(),
        cameraConfiguration.getCameraPosition().getZ(),
        Units.radiansToDegrees(cameraConfiguration.getCameraPosition().getRotation().getX()),
        Units.radiansToDegrees(cameraConfiguration.getCameraPosition().getRotation().getY()),
        Units.radiansToDegrees(cameraConfiguration.getCameraPosition().getRotation().getZ()));

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

      lastMegatag2Timestamp = estimate.timestampSeconds;

      return new VisionPoseEstimation(
          latencyUncompensatedPose,
          latencyCompensatedPose,
          getStdDevs(estimate),
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

    if (cornerPositions.length >= 8) {
      // 4 iterations bcuz we process 2 at a time
      for (int i = 0; i < 4; i++) {
        tagCorners.get(i).x = cornerPositions[i * 2];
        tagCorners.get(i).y = cornerPositions[i * 2 + 1];
      }
    }
  }

  /** Get standard deviations for the given pose estimate */
  protected OdometryStandardDevs getStdDevs(PoseEstimate estimate) {
    double[] doubleArray = LimelightExtras.getStandardDeviations(cameraName);

    return adjustStdDevsWithOdomPose(
        new OdometryStandardDevs(doubleArray[0], doubleArray[1], doubleArray[5]),
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
}
