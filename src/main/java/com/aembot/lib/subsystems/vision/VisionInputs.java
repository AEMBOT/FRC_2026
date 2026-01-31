package com.aembot.lib.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputs implements LoggableInputs {

  public boolean hasTag = false;

  public Rotation2d horizontalRotationToTag = new Rotation2d();

  public int tagID = -1;

  public List<Pose2d> tagCornerPositions;

  public double tagHeightPixels;

  public Rotation2d tagAngularSize;

  public double tagDistanceMeters;

  public Pose2d estimatedRobotPoseCompensated;

  public Pose2d estimatedRobotPoseUncompensated;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("HorizontalRotationToTag", horizontalRotationToTag);
    table.put("TagID", tagID);
    table.put("TagHeightPixels", tagHeightPixels);
    table.put("TagAngularSize", tagAngularSize);
    table.put("TagDistanceMeters", tagDistanceMeters);
    table.put("estimatedRobotPoseCompensated", estimatedRobotPoseCompensated);
    table.put("estimatedRobotPoseUncompensated", estimatedRobotPoseUncompensated);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    horizontalRotationToTag = table.get("HorizontalRotationToTag", horizontalRotationToTag);
    tagID = table.get("TagID", tagID);
    tagHeightPixels = table.get("TagHeightPixels", tagHeightPixels);
    tagAngularSize = table.get("TagAngularSize", tagAngularSize);
    tagDistanceMeters = table.get("TagDistanceMeters", tagDistanceMeters);
    estimatedRobotPoseCompensated =
        table.get("estimatedRobotPoseCompensated", estimatedRobotPoseCompensated);
    estimatedRobotPoseUncompensated =
        table.get("estimatedRobotPoseUncompensated", estimatedRobotPoseUncompensated);
  }
}
