package com.aembot.lib.subsystems.vision;

import com.aembot.lib.subsystems.vision.util.VisionStandardDevs;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputs implements LoggableInputs {

  public boolean hasTag = false;

  public int primaryTagID = -1;

  public VisionStandardDevs stdDevs;

  public Pose2d estimatedRobotPose;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("estimatedRobotPose", estimatedRobotPose);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    estimatedRobotPose = table.get("estimatedRobotPose", estimatedRobotPose);
  }
}
