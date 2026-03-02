package com.aembot.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Convert poses to transforms and vice versa */
public final class PositionUtil {
  /** Constants representing NaN values for various geometric types */
  public final class NaN {
    public static final Translation2d TRANSLATION2D = new Translation2d(Double.NaN, Double.NaN);
    public static final Translation3d TRANSLATION3D =
        new Translation3d(Double.NaN, Double.NaN, Double.NaN);

    public static final Rotation2d ROTATION2D = new Rotation2d(Double.NaN);
    public static final Rotation3d ROTATION3D = new Rotation3d(Double.NaN, Double.NaN, Double.NaN);

    public static final Pose2d POSE2D = new Pose2d(TRANSLATION2D, ROTATION2D);
    public static final Pose3d POSE3D = new Pose3d(TRANSLATION3D, ROTATION3D);

    public static final Transform2d TRANSFORM2D = new Transform2d(TRANSLATION2D, ROTATION2D);
    public static final Transform3d TRANSFORM3D = new Transform3d(TRANSLATION3D, ROTATION3D);
  }

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
