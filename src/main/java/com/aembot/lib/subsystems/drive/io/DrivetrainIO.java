package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

public interface DrivetrainIO {
  public void updateInputs(DrivetrainInputs inputs);

  /**
   * Log the states of the swerve drive modules in a more human-readable way, including their names
   * (ie. FL, BR)
   *
   * @param state Current Swerve Drive state to be logged
   */
  public void logModules(SwerveDriveState state, String prefix);

  /**
   * Resets the drive train odometry to the given pose. Ie. setting robot pose to auto starting
   * position
   *
   * @param pose
   */
  void resetOdometry(Pose2d pose);

  /**
   * Command the swerve drive to do something
   *
   * @param request The operation to preform on the swerve drive
   */
  void setRequest(SwerveRequest request);

  /**
   * Sets how much we trust the robots reported odometry. Stdevs increase as you trust the odometry
   * less
   *
   * @param xStd +/- X standard deviation in meters
   * @param yStd +/- Y standard deviation in meters
   * @param rotStd +/- θ standard deviation in radians
   */
  void setOdometryStdDevs(double xStd, double yStd, double rotStd);
}
