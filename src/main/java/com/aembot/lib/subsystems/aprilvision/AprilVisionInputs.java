package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.subsystems.aprilvision.util.CameraCalibration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AprilVisionInputs implements LoggableInputs {
  public boolean hasTag = false;
  public int tagID = -1;
  public Pose3d tagPosition = new Pose3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d());

  /** Horizontal rotation from the camera's center to the detected tag's center */
  public Rotation2d horizontalAngleToTag;

  /** Corner positions of the april tag */
  public List<Vector2> tagCornerPositions;

  /** The height of (<strong>not to</strong>) the currently detected tag in pixels. */
  public double tagHeightPixels;

  /**
   * The height of (<strong>not to</strong>) the currently detected tag as an angle.
   *
   * <p>In the below triangle, let 📷 be the camera, and the leg opposite to it be the tag. This is
   * on the xz plane, so below this is the floor.
   *
   * <pre>
   * .
   *    /|
   * 📷/_|
   * </pre>
   *
   * {@code tagHeightAngle} is the angle at the camera in this triangle.
   */
  public Rotation2d tagHeightAngle;

  /** Distance from the camera to the tag in meters */
  public double tagDistanceMeters;

  /**
   * Ms between the end of the exposure of the middle row of the sensor to the data being published
   * to network tables
   */
  public double latency;

  /**
   * The estimated pose of the robot based off the tag's data <strong>without</strong> compensation
   * for camera latency
   *
   * @see #robotPoseEstimationLatencyCompensated
   */
  public Pose2d robotPoseEstimationLatencyUncompensated;

  /**
   * The estimated pose of the robot based off the tag's data <strong>with</strong> compensation for
   * camera latency
   */
  public Pose2d robotPoseEstimationLatencyCompensated;

  /**
   * The calibration of the camera. This is io-layer specific because the calibration is typically
   * stored on a coprocessor
   */
  public CameraCalibration cameraCalibration;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("tagPosition", tagPosition);
    table.put("HorizontalAngleToTag", horizontalAngleToTag);
    table.put("TagID", tagID);
    table.put("TagHeightPixels", tagHeightPixels);
    table.put("TagHeightAngle", tagHeightAngle);
    table.put("TagDistanceMeters", tagDistanceMeters);
    table.put("RobotPoseEstimationLatencyUncompensated", robotPoseEstimationLatencyUncompensated);
    table.put("RobotPoseEstimationLatencyCompensated", robotPoseEstimationLatencyCompensated);
    table.put(
        "CameraCalibration",
        cameraCalibration != null ? cameraCalibration.serializeToString() : (String) null);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    tagPosition = table.get("tagPosition", tagPosition);
    horizontalAngleToTag = table.get("HorizontalAngleToTag", horizontalAngleToTag);
    tagID = table.get("TagID", tagID);
    tagHeightPixels = table.get("TagHeightPixels", tagHeightPixels);
    tagHeightAngle = table.get("TagHeightAngle", tagHeightAngle);
    tagDistanceMeters = table.get("TagDistanceMeters", tagDistanceMeters);
    robotPoseEstimationLatencyUncompensated =
        table.get(
            "RobotPoseEstimationLatencyUncompensated", robotPoseEstimationLatencyUncompensated);
    robotPoseEstimationLatencyCompensated =
        table.get("RobotPoseEstimationLatencyCompensated", robotPoseEstimationLatencyCompensated);

    cameraCalibration =
        CameraCalibration.deserializeFromString(table.get("CameraCalibration", (String) null));
  }
}
