package com.aembot.frc2026.constants.field;

import com.aembot.lib.constants.fields.YearFieldConstantable;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class Field2026 implements YearFieldConstantable {
  /** Hub center used by the shooting tables, in the blue-origin field frame. */
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6101, 4.03479);

  private static Field2026 INSTANCE = new Field2026();

  public static Field2026 get() {
    return INSTANCE;
  }

  public final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  @Override
  public AprilTagFieldLayout getFieldLayout() {
    return APRIL_TAG_FIELD_LAYOUT;
  }
}
