package com.aembot.frc2026.util;

import com.aembot.lib.math.ConcurrentInterpolatable2DMap;
import com.opencsv.bean.CsvBindByName;
import com.opencsv.bean.CsvToBeanBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.Velocity;

import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

/** Class that parses a csv file to an interpolatable 2D Map */
public class OptimalVelocityTable extends ConcurrentInterpolatable2DMap<Translation3d> {

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

  /**
   * Create a table of optimal velocities from a csv file
   *
   * @param filePath path to the csv file to parse frome
   */
  public OptimalVelocityTable(String filePath) {

    super(Translation3d::interpolate);

    try (FileReader reader = new FileReader(filePath)) {

      List<DataRow> data = new CsvToBeanBuilder<DataRow>(reader).build().parse();

      for (DataRow row : data) {
        addPoint(
            row.xPosition,
            row.yPosition,
            new Translation3d(row.xVelocity, row.yVelocity, row.zVelocity));
      }

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Override so that xpos and ypos are used as arguments instead of q1 and q2
  @Override
  public Optional<Translation3d> getPoint(double xPos, double yPos) {
    return super.getPoint(xPos, yPos);
  }

  /**
   * Get the optimal velocity for a given point.
   *
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return value of the optimal velocity at the sampled point
   */
  public Translation3d getVelocity(double xPos, double yPos) {
    return getPoint(xPos, yPos).orElseThrow();
  }

  /**
   * Get a rotation representing the direction of the optimal velocity at a given point
   * 
   * @param xPos x value of the point to sample
   * @param yPos y value of the point to sample
   * @return direction of the shooter at the sampled point
   */
  public Rotation3d getVelocityDirection(double xPos, double yPos) {
    return new Rotation3d(getPoint(xPos, yPos).orElseThrow().toVector());
  }
}
