package com.aembot.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Convert poses to transforms and vice versa */
public final class PositionUtil {
  public static Pose3d toPose3d(Transform3d transform3d) {
    return new Pose3d(transform3d.getTranslation(), transform3d.getRotation());
  }

  public static Transform3d toTransform3d(Pose3d pose3d) {
    return new Transform3d(pose3d.getTranslation(), pose3d.getRotation());
  }

  public static Pose2d toPose2d(Transform2d transform2d) {
    return new Pose2d(transform2d.getTranslation(), transform2d.getRotation());
  }

  public static Transform2d toTransform2d(Pose2d pose2d) {
    return new Transform2d(pose2d.getTranslation(), pose2d.getRotation());
  }
}
