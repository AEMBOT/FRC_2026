package com.aembot.lib.subsystems.aprilvision.util;

/**
 * Collection of static methods not provided by official {@link LimelightHelpers}
 *
 * @see LimelightHelpers
 */
public class LimelightExtras {
  /**
   * Get the standard deviations of the pose estimate as a double array
   *
   * @param limelightName Name of the Limelight camera as it appears on NetworkTables
   * @return A double array in the format {xStdDev, yStdDev, zStdDev, rollStdDev, pitchStdDev,
   *     yawStdDev}
   */
  public static double[] getStandardDeviations(String limelightName) {
    return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "stddevs");
  }

  public static double getCameraTemperature(String limelightName) {
    double[] hardwareData = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "hw");

    if (hardwareData.length < 4) return -1;

    return hardwareData[0];
  }
}
