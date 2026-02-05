package com.aembot.lib.subsystems.vision;

import com.aembot.lib.subsystems.vision.util.VisionStandardDeviations;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputs implements LoggableInputs {

  public boolean hasTag = false;

  public int primaryTagID = -1;

  public int numTags = 0;

  public VisionStandardDeviations stdDevs = VisionStandardDeviations.ifSeesNoTags();

  public Pose2d estimatedRobotPose = Pose2d.kZero;

  public double lastEstimateTimestamp = 0;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("NumTags", numTags);
    table.put("estimatedRobotPose", estimatedRobotPose);
    table.put("StandardDeviations", stdDevs.asDoubleArray());
    table.put("LastEstimateTimestamp", lastEstimateTimestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    numTags = table.get("NumTags", numTags);
    estimatedRobotPose = table.get("estimatedRobotPose", estimatedRobotPose);
    stdDevs =
        VisionStandardDeviations.fromDoubleArray(
            table.get("StandardDeviations", stdDevs.asDoubleArray()));
    lastEstimateTimestamp = table.get("LastEstimateTimestamp", lastEstimateTimestamp);
  }
}
