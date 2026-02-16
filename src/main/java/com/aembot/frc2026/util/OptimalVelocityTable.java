package com.aembot.frc2026.util;

import com.aembot.lib.math.ConcurrentInterpolatable2DMap;
import com.opencsv.bean.CsvBindByName;
import com.opencsv.bean.CsvToBeanBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Class that parses a csv file to an interpolatable 2D Map */
public class OptimalVelocityTable extends ConcurrentInterpolatable2DMap<Translation3d> {

  //
  private final String fileName;

  // Internal class used for reading csv file
  public static class DataRow {

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

      List<DataRow> data =
          new CsvToBeanBuilder<DataRow>(reader)
              .withIgnoreLeadingWhiteSpace(true)
              .withType(DataRow.class)
              .build()
              .parse();

      for (DataRow row : data) {
        addPoint(
            row.xPosition,
            row.yPosition,
            new Translation3d(row.xVelocity, row.yVelocity, row.zVelocity));
      }

    } catch (IOException e) {
      e.printStackTrace();
    }

    String[] directoryList = filePath.split("/");

    this.fileName = directoryList[directoryList.length - 1];
  }

  // Override so that xpos and ypos are used as arguments instead of q1 and q2
  @Override
  public Optional<Translation3d> getPoint(double xPos, double yPos) {
    return super.getPoint(xPos, yPos);
  }

  /**
   * Get the optimal velocity for a given pose and velocity
   *
   * @param robotPose current pose of the robot, only uses x and y parts
   * @param fieldRelativeChassisSpeeds speed of the robot relative to the field
   * @return value of the optimal velocity at the sampled point
   */
  public Translation3d getFuelInitVelocity(
      Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {

    ChassisSpeeds absoluteChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(fieldRelativeChassisSpeeds, robotPose.getRotation());

    Translation3d velocity =
        getPoint(robotPose.getX(), robotPose.getY())
            .orElse(Translation3d.kZero)
            .minus(
                new Translation3d(
                    absoluteChassisSpeeds.vxMetersPerSecond,
                    absoluteChassisSpeeds.vyMetersPerSecond,
                    0));
    Logger.recordOutput(
        "AutoAim/" + fileName,
        new Pose3d(velocity.plus(new Translation3d(robotPose.getTranslation())), Rotation3d.kZero));
    return velocity;
  }

  /**
   * Get a rotation representing the direction of the optimal velocity of fuel at a given pose and
   * velocity
   *
   * @param robotPose current pose of the robot, only uses x and y parts
   * @param fieldRelativeChassisSpeeds speed of the robot relative to the field
   * @return direction of the shooter at the sampled point
   */
  public Rotation3d getFuelInitVelocityRotation3d(
      Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {

    Translation3d velocity = getFuelInitVelocity(robotPose, fieldRelativeChassisSpeeds);

    double yaw = Math.atan2(velocity.getY(), velocity.getX());

    double pitch =
        (Math.PI / 2) - Math.acos(velocity.getZ() / velocity.getDistance(Translation3d.kZero));

    return new Rotation3d(0, pitch, yaw);
  }

  /**
   * Get the magnitude of the the optimal velocity at a given pose and velocity
   *
   * @param robotPose current pose of the robot, only uses x and y parts
   * @param fieldRelativeChassisSpeeds speed of the robot relative to the field
   * @return exit speed of fuel at the sampled point
   */
  public double getFuelInitVelocityMagnitude(
      Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {
    return getFuelInitVelocity(robotPose, fieldRelativeChassisSpeeds)
        .getDistance(Translation3d.kZero);
  }
}
