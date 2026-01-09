package com.aembot.lib.constants;

import com.aembot.lib.constants.fields.YearFieldConstantable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  /** The height dimension (NOT POSITION) of an april tag in meters. */
  public static final double APRIL_TAG_HEIGHT_METERS = Units.inchesToMeters(6.5);

  /** The width dimension of an april tag in meters. */
  public static final double APRIL_TAG_WIDTH_METERS = Units.inchesToMeters(6.5);

  public static Pose3d getAprilTagPose3d(int id, YearFieldConstantable field) {
    if (0 > id || id > field.getNumTags()) {
      throw new IllegalArgumentException(
          String.format("ID must be between 1 and %d. Was %d", field.getNumTags(), id));
    }

    return field
        .getFieldLayout()
        .getTagPose(id)
        .orElseThrow(
            () -> {
              throw new RuntimeException(
                  String.format("getTagPose called for unexpected tag %d", id));
            });
  }

  public static Pose2d getAprilTagPose2d(int id, YearFieldConstantable field) {
    return getAprilTagPose3d(id, field).toPose2d();
  }
}
