package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.core.phoenix6.AEMSwerveDriveState;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;

public interface DrivetrainIO {
  public void updateInputs(DrivetrainInputs inputs);

  /**
   * Log the states of the swerve drive modules in a more human-readable way, including their names
   * (ie. FL, BR)
   *
   * @param inputs Current Drivetrain inputs containing state to be logged
   */
  public void logModules(DrivetrainInputs inputs, String prefix);

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

  void addVisionEstimation(AprilCameraOutput cameraOutput);

  /**
   * Register a listener that receives every swerve drive state as soon as it is produced, rather
   * than once per robot loop.
   *
   * <p>On hardware the listener runs on the drivetrain odometry thread at 250 Hz. It must not log
   * (AEMLogger is not thread safe), must not block, and must finish in microseconds or it will
   * delay odometry. Implementations with no odometry thread ignore the listener.
   *
   * @param listener Consumer of the latest drivetrain state
   */
  default void registerFastStateListener(Consumer<AEMSwerveDriveState> listener) {}
}
