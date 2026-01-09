package com.aembot.lib.constants.fields;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

// TODO Determine if this interface is useful

/** Interface for describing a field. */
public interface YearFieldConstantable {
  public AprilTagFieldLayout getFieldLayout();

  /** Get the number of tags in the field */
  public default int getNumTags() {
    final int numTags = getFieldLayout().getTags().size();
    return numTags;
  }
}
