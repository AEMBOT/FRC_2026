package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.dyn4j.geometry.Vector2;

public class Limelight4IOHardware extends LimelightIO {
  /* ---- NETWORK TABLES ENTRIES ---- */
  /** bool NT entry indicating whether we have a tag targetted. "tv" on network table */
  protected final NetworkTableEntry validTagEntry;

  /**
   * double NT entry indicating horizontal offset in degrees from crosshair to target. "tx" on
   * network table
   */
  protected final NetworkTableEntry xOffsetEntry;

  /** int NT entry indicating id of the targeted apriltag. "tid" on network table */
  protected final NetworkTableEntry tagIDEntry;

  /**
   * double[] NT entry indicating corner coordinates in pixels [x0,y0,x1,y1......]. "tcornxy" on
   * network table.
   */
  protected final NetworkTableEntry tagCornerPositionsEntry;

  /**
   * int NT entry indicating number of frames to skip between processed frames to reduce temperature
   * rise. "throttle_set" on network table
   */
  protected final NetworkTableEntry throttleSetEntry;

  /**
   * double NT entry indicating capture pipeline latency (ms). Time between the end of the exposure
   * of the middle row of the sensor to the beginning of the tracking pipeline. Sum with {@link
   * #pipelineLatencyEntry} to get total latency. "cl" on network table.
   */
  protected final NetworkTableEntry captureLatencyEntry;

  /**
   * double NT entry indicating The pipeline's latency contribution (ms). Sum with {@link
   * #captureLatencyEntry} to get total latency. "tl" on network table (for some reason).
   */
  protected final NetworkTableEntry pipelineLatencyEntry;

  /* ---- END NETWORK TABLES ENTRIES ---- */

  protected final CameraConfiguration cameraConfiguration;
  protected final YearFieldConstantable fieldConstants;

  protected final List<Vector2> tagCorners =
      List.of(new Vector2(), new Vector2(), new Vector2(), new Vector2());

  protected final RobotState robotStateInstance;

  public Limelight4IOHardware(
      CameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance) {
    this.cameraConfiguration = config;
    this.fieldConstants = fieldConstants;
    this.robotStateInstance = robotStateInstance;

    NetworkTable networkTable =
        NetworkTableInstance.getDefault().getTable("limelight-" + cameraConfiguration.cameraName);

    validTagEntry = networkTable.getEntry("tv");
    xOffsetEntry = networkTable.getEntry("tx");
    tagIDEntry = networkTable.getEntry("tid");
    tagCornerPositionsEntry = networkTable.getEntry("tcornxy");
    throttleSetEntry = networkTable.getEntry("throttle_set");
    captureLatencyEntry = networkTable.getEntry("cl");
    pipelineLatencyEntry = networkTable.getEntry("tl");
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    inputs.latency = captureLatencyEntry.getDouble(0) + pipelineLatencyEntry.getDouble(0);

    inputs.hasTag = validTagEntry.getInteger(0) == 1;
    inputs.tagID = (int) tagIDEntry.getInteger(-1);
    inputs.horizontalAngleToTag = Rotation2d.fromDegrees(xOffsetEntry.getDouble(0));

    updateCornerPositions();
    inputs.tagCornerPositions = this.tagCorners;

    estimatePoseAndPublish(inputs);
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
    if (inputs.hasTag) {
      Pose3d tagPose = fieldConstants.getAprilTagPose3d(inputs.tagID);

      inputs.tagHeightPixels = computeTagHeightInPixels(inputs.tagCornerPositions);
      inputs.tagHeightAngle = computeTagHeightInRotations(inputs.tagHeightPixels);

      inputs.tagDistanceMeters =
          computeDistanceToTagMeters(inputs.tagHeightAngle, tagPose.getZ())
              * getConfiguration().cameraDistanceScalar;

      // If tag id valid
      if (inputs.tagID >= 1 && inputs.tagID <= fieldConstants.getNumTags()) {
        Rotation2d robotRotation = robotStateInstance.getLatestFieldRobotPose().getRotation();

        Translation2d robotToTagTranslation =
            computeRobotToTag(robotRotation, inputs.horizontalAngleToTag, inputs.tagDistanceMeters);

        VisionPoseEstimation poseEstimation =
            computeRobotPose(
                tagPose.toPose2d(),
                robotRotation,
                robotToTagTranslation,
                robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
                inputs.latency);

        inputs.robotPoseEstimationLatencyUncompensated = poseEstimation.latencyUncompensatedPose();
        inputs.robotPoseEstimationLatencyCompensated = poseEstimation.latencyCompensatedPose();
      }
    }
  }

  /**
   * Convert the Limelight-provided double[] of corner coordinates into a list of Vector2s
   * representing the corner positions in pixels
   */
  private void updateCornerPositions() {
    double[] cornerPositions = tagCornerPositionsEntry.getDoubleArray(new double[0]);

    if (cornerPositions.length == 8) {
      // 4 iterations bcuz we process 2 at a time
      for (int i = 0; i < 4; i++) {
        tagCorners.get(i).x = cornerPositions[i * 2];
        tagCorners.get(i).y = cornerPositions[i * 2 + 1];
      }
    }
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return cameraConfiguration;
  }
}
