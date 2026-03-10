package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.math.PositionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.opencv.core.Point;

public class AprilVisionInputs implements LoggableInputs {
  public boolean hasTag = false;
  public int tagID = -1;
  public Pose3d tagPosition = PositionUtil.NaN.POSE3D;

  /** Horizontal rotation from the camera's center to the detected tag's center */
  public Rotation2d horizontalAngleToTag;

  /** Corner positions of the april tag */
  public List<Point> tagCornerPositions;

  /**
   * Ms between the end of the exposure of the middle row of the sensor to the data being published
   * to network tables
   */
  public double latency;

  /**
   * The estimated pose of the robot from the vision coprocessor. Ie. Limelight's MegaTag 2. Not
   * latency compensated.
   */
  public Pose2d coprocessorEstimationLatencyUncompensated;

  public OdometryStandardDevs coprocessorEstimationStdDevs;

  public double coprocessorPoseEstimationAmbiguity;

  public double coprocessorEstimationTimestamp;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("tagPosition", tagPosition);
    table.put("HorizontalAngleToTag", horizontalAngleToTag);
    table.put("TagID", tagID);
    table.put(
        "CoprocessorEstimationLatencyUncompensated", coprocessorEstimationLatencyUncompensated);
    table.put("CoprocessorEstimationStdDevs", coprocessorEstimationStdDevs);
    table.put("CoprocessorPoseEstimationAmbiguity", coprocessorPoseEstimationAmbiguity);
    table.put("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    tagPosition = table.get("tagPosition", tagPosition);
    horizontalAngleToTag = table.get("HorizontalAngleToTag", horizontalAngleToTag);
    tagID = table.get("TagID", tagID);
    coprocessorEstimationLatencyUncompensated =
        table.get(
            "CoprocessorEstimationLatencyUncompensated", coprocessorEstimationLatencyUncompensated);
    coprocessorEstimationStdDevs =
        table.get("CoprocessorEstimationStdDevs", coprocessorEstimationStdDevs);
    coprocessorPoseEstimationAmbiguity =
        table.get("CoprocessorPoseEstimationAmbiguity", coprocessorPoseEstimationAmbiguity);
    coprocessorEstimationTimestamp =
        table.get("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }
}
