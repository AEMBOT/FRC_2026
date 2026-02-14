package com.aembot.lib.constants.fields;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Interface for describing a field. */
public interface YearFieldConstantable {
  public AprilTagFieldLayout getFieldLayout();

  /** Get the number of tags in the field */
  public default int getNumTags() {
    final int numTags = getFieldLayout().getTags().size();
    return numTags;
  }

  public default Pose3d getAprilTagPose3d(int id) {
    if (0 > id || id > this.getNumTags()) {
      throw new IllegalArgumentException(
          String.format("ID must be between 1 and %d. Was %d", this.getNumTags(), id));
    }

    return this.getFieldLayout()
        .getTagPose(id)
        .orElseThrow(
            () -> {
              throw new RuntimeException(
                  String.format("getTagPose called for unexpected tag %d", id));
            });
  }

  public default Pose2d getAprilTagPose2d(int id) {
    return getAprilTagPose3d(id).toPose2d();
  }
}
