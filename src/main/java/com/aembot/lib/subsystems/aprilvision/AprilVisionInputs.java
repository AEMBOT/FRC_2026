package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.math.PositionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AprilVisionInputs implements LoggableInputs {
  public boolean hasTag = false;
  public int tagID = -1;
  public Pose3d tagPosition = PositionUtil.NaN.POSE3D;

  /**
   * Ms between the end of the exposure of the middle row of the sensor to the data being published
   * to network tables
   */
  public double latency;

  /**
   * The estimated pose of the robot from the vision coprocessor. Ie. Limelight's MegaTag 2. Not
   * latency compensated.
   */
  public Pose2d coprocessorEstimationLatencyUncompensated = Pose2d.kZero;

  /**
   * The estimated pose of the robot from the vision coprocessor. Ie. Limelight's MegaTag 2. Latency
   * compensated.
   */
  public Pose2d coprocessorEstimationLatencyCompensated = Pose2d.kZero;

  public OdometryStandardDevs coprocessorEstimationStdDevs = new OdometryStandardDevs(0, 0, 0);

  public double coprocessorPoseEstimationAmbiguity;

  public double coprocessorEstimationTimestamp;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("tagPosition", tagPosition);
    table.put("TagID", tagID);
    table.put(
        "CoprocessorEstimationLatencyUncompensated", coprocessorEstimationLatencyUncompensated);
    table.put("CoprocessorEstimationLatencyCompensated", coprocessorEstimationLatencyCompensated);
    table.put("CoprocessorEstimationStdDevs", coprocessorEstimationStdDevs);
    table.put("CoprocessorPoseEstimationAmbiguity", coprocessorPoseEstimationAmbiguity);
    table.put("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    tagPosition = table.get("tagPosition", tagPosition);
    tagID = table.get("TagID", tagID);
    coprocessorEstimationLatencyUncompensated =
        table.get(
            "CoprocessorEstimationLatencyUncompensated", coprocessorEstimationLatencyUncompensated);
    coprocessorEstimationLatencyCompensated =
        table.get(
            "CoprocessorEstimationLatencyCompensated", coprocessorEstimationLatencyCompensated);
    coprocessorEstimationStdDevs =
        table.get("CoprocessorEstimationStdDevs", coprocessorEstimationStdDevs);
    coprocessorPoseEstimationAmbiguity =
        table.get("CoprocessorPoseEstimationAmbiguity", coprocessorPoseEstimationAmbiguity);
    coprocessorEstimationTimestamp =
        table.get("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
  }
}
