package com.aembot.lib.subsystems.aprilvision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;

/** Record representing the calibration of a camera to account for lens distortion */
public record CameraCalibration(Matrix<N3, N3> cameraMatrix, Matrix<N8, N1> distCoeffs) {
  // This serialization method is not efficient, but it's only going to be used at startup on the
  // real robot so it shouldn't matter too much... more pressing is that it's not very readable in
  // logs...
  // TODO BEFORE MERGE: figure out a way to make this more human-readable (and hopefully efficient)
  // (possibly nested arrays?)
  /**
   * Serialize the camera calibration to a string for logging purposes.
   *
   * @return The camera calibration as a string in the format
   *     "11,12,13,21,22,23,31,32,33:11,21,31,41,51,61,71,81", where the digits are replaced with
   *     matrix elements. Numbers prior to the colon are in {@link #cameraMatrix()} and after it are
   *     in {@link #distCoeffs()}.
   * @see #deserializeFromString(String)
   */
  public String serializeToString() {
    StringBuilder builder = new StringBuilder();

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        builder.append(cameraMatrix().get(r, c));
        if (r != 2 && c != 2) builder.append(",");
      }
    }

    builder.append(":");

    for (int r = 0; r < 8; r++) {
      builder.append(distCoeffs().get(r, 0));
      if (r != 7) builder.append(",");
    }

    return builder.toString();
  }

  /**
   * Deserialize the camera calibration from a string.
   *
   * @param serialized A string in the format "11,12,13,21,22,23,31,32,33:11,21,31,41,51,61,71,81",
   *     where the digits are replaced with matrix elements. Numbers prior to the colon are in
   *     {@link #cameraMatrix()} and after it are in {@link #distCoeffs()}.
   * @return The camera calibration
   */
  public static CameraCalibration deserializeFromString(String serialized) {
    // I barely know regex so this is messy but should work I think
    if (!serialized.matches("(?:[\\d|.]+,){8}[\\d|.]:(?:[\\d|.],){7}\\d")) {
      DriverStation.reportError(
          "Error while deserializing camera calibration. Given string \""
              + serialized
              + "\" is incorrectly formatted!",
          true);
      return null;
    }

    String[] split = serialized.split(":");

    // 3x3 matrix
    var cameraMatrix = new Matrix<>(Nat.N3(), Nat.N3());

    String[] cameraMatrixString = split[0].split(",");

    for (int i = 0; i < 9; i++) {
      cameraMatrix.set(i % 3, Math.floorDiv(i, 3), Double.parseDouble(cameraMatrixString[i]));
    }

    // 8x1 matrix
    var distCoeffs = new Matrix<>(Nat.N8(), Nat.N1());

    String[] distCoeffsString = split[1].split(",");

    for (int i = 0; i < 8; i++) {
      distCoeffs.set(i, 0, Double.parseDouble(distCoeffsString[i]));
    }

    return new CameraCalibration(cameraMatrix, distCoeffs);
  }

  @Override
  public String toString() {
    return this.serializeToString();
  }
}
