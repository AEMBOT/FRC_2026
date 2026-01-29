package com.aembot.frc2026.constants.field;

import com.aembot.lib.constants.fields.YearFieldConstantable;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Field2025 implements YearFieldConstantable {
  private static Field2025 INSTANCE = new Field2025();

  public static Field2025 get() {
    return INSTANCE;
  }

  public final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  @Override
  public AprilTagFieldLayout getFieldLayout() {
    return APRIL_TAG_FIELD_LAYOUT;
  }
}
