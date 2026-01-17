package com.aembot.lib.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

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
  void setControl(SwerveRequest request);

  /**
   * Apply the SwerveRequest in the supplier to the swerve drive train until this command is
   * cancelled
   *
   * @param requestSupplier The Supplier<SwerveRequest> used to control what request is being given
   *     to the drive train
   * @param subsystemsRequired The subsystems required to drive this command (will be whatever the
   *     drive subsystem is)
   * @return The command to be run
   */
  Command continuousRequestCommand(
      Supplier<SwerveRequest> requestSupplier, Subsystem... subsystemsRequired);

  /**
   * Sets how much we trust the robots reported odometry. Stdevs increase as you trust the odometry
   * less
   *
   * @param xStd +/- X standard deviation in meters
   * @param yStd +/- Y standard deviation in meters
   * @param rotStd +/- θ standard deviation in radians
   */
  void setOdometryStdDevs(double xStd, double yStd, double rotStd);

  /**
   * Retrieve the swerve drive kinematics for this swerve drivetrain
   *
   * @return The SwerveDriveKinematics object for this swerve drive train
   */
  SwerveDriveKinematics getSwerveKinematics();
}
