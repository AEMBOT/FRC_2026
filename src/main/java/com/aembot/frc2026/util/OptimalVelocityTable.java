package com.aembot.frc2026.util;

import com.aembot.lib.math.ConcurrentInterpolatable2DMap;
import com.opencsv.bean.CsvBindByName;
import com.opencsv.bean.CsvToBeanBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

/** Class that parses a csv file to an interpolatable 2D Map */
public class OptimalVelocityTable extends ConcurrentInterpolatable2DMap<Double[]> {

  // Internal class used for reading csv file
  private class DataRow {

    @CsvBindByName(column = "X Position")
    public double xPosition;

    @CsvBindByName(column = "Y Position")
    public double yPosition;

    @CsvBindByName(column = "Z Position")
    public double zPosition;

    @CsvBindByName(column = "X Velocity")
    public double xVelocity;

    @CsvBindByName(column = "Y Velocity")
    public double yVelocity;

    @CsvBindByName(column = "Z Velocity")
    public double zVelocity;
  }

  // Function used to linearly interpolate between 2 3d popnts
  static Interpolator<Double[]> interpolationFunc =
      (p1, p2, t) -> {
        return new Double[] {
          MathUtil.interpolate(p1[0], p2[0], t),
          MathUtil.interpolate(p1[1], p2[1], t),
          MathUtil.interpolate(p1[2], p2[2], t)
        };
      };

  /**
   * Create a table of optimal velocities from a csv file
   *
   * @param filePath path to the csv file to parse frome
   */
  public OptimalVelocityTable(String filePath) {

    super(interpolationFunc);

    try (FileReader reader = new FileReader(filePath)) {

      List<DataRow> data = new CsvToBeanBuilder<DataRow>(reader).build().parse();

      for (DataRow row : data) {
        addPoint(
            row.xPosition,
            row.yPosition,
            new Double[] {row.xVelocity, row.yVelocity, row.zVelocity});
      }

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Override so that xpos and ypos are used as arguments instead of q1 and q2
  @Override
  public Optional<Double[]> getPoint(double xPos, double yPos) {
    return super.getPoint(xPos, yPos);
  }

  /**
   * Get the optimal x velocity for a given point.
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return x value of the optimal velocity at the sampled point
   */
  public Double getXVelocity(double xPos, double yPos) {
    Double[] velocity = getPoint(xPos, yPos).orElseThrow();
    return velocity[0];
  }

  /**
   * Get the optimal y velocity for a given point.
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return y value of the optimal velocity at the sampled point
   */
  public Double getYVelocity(double xPos, double yPos) {
    Double[] velocity = getPoint(xPos, yPos).orElseThrow();
    return velocity[1];
  }

  /**
   * Get the optimal z velocity for a given point.
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return z value of the optimal velocity at the sampled point
   */
  public Double getZVelocity(double xPos, double yPos) {
    Double[] velocity = getPoint(xPos, yPos).orElseThrow();
    return velocity[2];
  }

  /**
   * Get the optimal yaw in degrees from the x axis
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return optimal yaw
   */
  public Double getYawDegrees(double xPos, double yPos) {
    Double[] velocity = getPoint(xPos, yPos).orElseThrow();
    return Math.toDegrees(Math.atan2(velocity[1], velocity[2]));
  }

  /**
   * Get the optimal pitch in degrees from horizontal
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return optimal pitch
   */
  public Double getPitchDegrees(double xPos, double yPos) {
    Double[] velocity = getPoint(xPos, yPos).orElseThrow();

    Double velocityMagnitutde =
        Math.sqrt(Math.pow(velocity[0], 2) + Math.pow(velocity[1], 2) + Math.pow(velocity[2], 2));

    return 180 - Math.toDegrees(Math.acos(velocity[2] / velocityMagnitutde));
  }
}
