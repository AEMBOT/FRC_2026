package com.aembot.lib.subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Helper class to store standard deviation values for robot pose */
public class VisionStandardDeviations {

  /** x component of the standard deviation in position */
  public final double xStdDev;

  /** y component of the standard deviation in position */
  public final double yStdDev;

  /** z component of the standard deviation in position */
  public final double zStdDev;

  /** roll component of the standard devian in rotation */
  public final double rollStdDev;

  /** pitch component of the standard devian in rotation */
  public final double pitchStdDev;

  /** yaw component of the standard devian in rotation */
  public final double yawStdDev;

  /**
   * Create a new storage class for standard deviations
   *
   * @param xStdDev x component of the standard deviation in position
   * @param yStdDev y component of the standard deviation in position
   * @param zStdDev z component of the standard deviation in position
   * @param rollStdDev roll component of the standard devian in rotation
   * @param pitchStdDev pitch component of the standard devian in rotation
   * @param yawStdDev yaw component of the standard devian in rotation
   */
  public VisionStandardDeviations(
      double xStdDev,
      double yStdDev,
      double zStdDev,
      double rollStdDev,
      double pitchStdDev,
      double yawStdDev) {
    this.xStdDev = xStdDev;
    this.yStdDev = yStdDev;
    this.zStdDev = zStdDev;
    this.rollStdDev = rollStdDev;
    this.pitchStdDev = pitchStdDev;
    this.yawStdDev = yawStdDev;
  }

  /**
   * Create a new standard deviations object from an array of values
   *
   * @param stdDevsArray array of standard deviations
   * @return new VisionStandardDeviations object
   */
  public static VisionStandardDeviations fromDoubleArray(double[] stdDevsArray) {
    return new VisionStandardDeviations(
        stdDevsArray[0],
        stdDevsArray[1],
        stdDevsArray[2],
        stdDevsArray[3],
        stdDevsArray[4],
        stdDevsArray[5]);
  }

  /**
   * Create a new VisionStandardDeviations object with arbitrarily high value
   *
   * <p>Meant to be used to generate std devs for an estimate that sees no tags
   *
   * @return new VisionStandardDeviation object
   */
  public static VisionStandardDeviations ifSeesNoTags() {
    return new VisionStandardDeviations(
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        Double.MAX_VALUE);
  }

  /**
   * @return Standard deviations as a three value matrix
   */
  public Matrix<N3, N1> asMatrix() {
    return VecBuilder.fill(xStdDev, yStdDev, yawStdDev);
  }

  /**
   * @return Standard deviations as a double array
   */
  public double[] asDoubleArray() {
    return new double[] {xStdDev, yStdDev, zStdDev, rollStdDev, pitchStdDev, yawStdDev};
  }

  /**
   * @return Standard deviations of the x position
   */
  public double getXStdDev() {
    return xStdDev;
  }

  /**
   * @return Standard deviations of the y position
   */
  public double getYStdDev() {
    return yStdDev;
  }

  /**
   * @return Standard deviations of the y position
   */
  public double getZStdDev() {
    return zStdDev;
  }

  /**
   * @return Standard deviations of the roll angle
   */
  public double getRollStdDev() {
    return rollStdDev;
  }

  /**
   * @return Standard deviations of the pitch angle
   */
  public double getPitchStdDev() {
    return pitchStdDev;
  }

  /**
   * @return Standard deviations of the yaw angle
   */
  public double getYawStdDev() {
    return yawStdDev;
  }
}
