package com.aembot.lib.config.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Represents standard deviations for odometry */
public record OdometryStandardDevs(double xStdDev, double yStdDev, double rotStdDev) {
  /**
   * @return 3x1 matrix of
   *     <pre>{@code <xStdDev(), yStdDev(), rotStdDev()>}</pre>
   */
  public Matrix<N3, N1> toMatrix() {
    return VecBuilder.fill(xStdDev(), yStdDev(), rotStdDev());
  }
}
