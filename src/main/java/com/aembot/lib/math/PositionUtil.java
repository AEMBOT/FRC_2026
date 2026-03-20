package com.aembot.lib.math;

import com.aembot.frc2026.constants.field.Field2026;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  public final class RotationConstants {
    public static final Rotation3d ROT_3D_180_DEG = new Rotation3d(Rotation2d.k180deg);
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

  public static Translation3d flipForAlliance(Translation3d bluePos) {
    switch (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue)) {
      case Red:
        return new Translation3d(
            Field2026.get().getFieldLayout().getFieldLength() - bluePos.getX(),
            Field2026.get().getFieldLayout().getFieldWidth() - bluePos.getY(),
            bluePos.getZ());
      case Blue:
      default:
        return bluePos;
    }
  }

  public static Translation2d flipForAlliance(Translation2d bluePos) {
    switch (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue)) {
      case Red:
        return new Translation2d(
            Field2026.get().getFieldLayout().getFieldLength() - bluePos.getX(),
            Field2026.get().getFieldLayout().getFieldWidth() - bluePos.getY());
      case Blue:
      default:
        return bluePos;
    }
  }

  public static Pose3d flipForAlliance(Pose3d bluePos) {
    switch (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue)) {
      case Red:
        return new Pose3d(
            Field2026.get().getFieldLayout().getFieldLength() - bluePos.getX(),
            Field2026.get().getFieldLayout().getFieldWidth() - bluePos.getY(),
            bluePos.getZ(),
            bluePos.getRotation().plus(RotationConstants.ROT_3D_180_DEG));
      case Blue:
      default:
        return bluePos;
    }
  }

  public static Pose2d flipForAlliance(Pose2d bluePos) {
    switch (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue)) {
      case Red:
        return new Pose2d(
            Field2026.get().getFieldLayout().getFieldLength() - bluePos.getX(),
            Field2026.get().getFieldLayout().getFieldWidth() - bluePos.getY(),
            bluePos.getRotation().plus(Rotation2d.k180deg));
      case Blue:
      default:
        return bluePos;
    }
  }
}
