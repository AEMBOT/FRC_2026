package com.aembot.lib.subsystems.aprilvision;

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

  // ===== RAW COPROCESSOR DATA (for replay) =====

  /**
   * Raw pose estimate from the vision coprocessor (e.g., Limelight MegaTag2). This is the
   * unprocessed pose before any RIO-side transformations.
   */
  public Pose2d rawCoprocessorPose;

  /** Raw standard deviations array from the coprocessor (12-element array for Limelight) */
  public double[] rawStdDevsArray = new double[0];

  /** Average distance to detected tags in meters */
  public double avgTagDist;

  /** Average tag area as percentage of image (0-100) */
  public double avgTagArea;

  /** Number of tags detected */
  public int tagCount;

  /** Timestamp of the coprocessor estimate in seconds */
  public double coprocessorEstimationTimestamp;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("tagPosition", tagPosition);
    table.put("HorizontalAngleToTag", horizontalAngleToTag);
    table.put("TagID", tagID);
    table.put("RawCoprocessorPose", rawCoprocessorPose);
    table.put("RawStdDevsArray", rawStdDevsArray);
    table.put("AvgTagDist", avgTagDist);
    table.put("AvgTagArea", avgTagArea);
    table.put("TagCount", tagCount);
    table.put("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    tagPosition = table.get("tagPosition", tagPosition);
    horizontalAngleToTag = table.get("HorizontalAngleToTag", horizontalAngleToTag);
    tagID = table.get("TagID", tagID);
    rawCoprocessorPose = table.get("RawCoprocessorPose", rawCoprocessorPose);
    rawStdDevsArray = table.get("RawStdDevsArray", rawStdDevsArray);
    avgTagDist = table.get("AvgTagDist", avgTagDist);
    avgTagArea = table.get("AvgTagArea", avgTagArea);
    tagCount = table.get("TagCount", tagCount);
    coprocessorEstimationTimestamp =
        table.get("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }
}
