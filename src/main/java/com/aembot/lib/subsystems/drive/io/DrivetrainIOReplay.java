package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

/** No-op implementation of {@link DrivetrainIO} for replay */
public class DrivetrainIOReplay implements DrivetrainIO {
  @Override
  public void updateInputs(DrivetrainInputs inputs) {}

  @Override
  public void logModules(DrivetrainInputs inputs, String prefix) {}

  @Override
  public void resetOdometry(Pose2d pose) {}

  @Override
  public void setRequest(SwerveRequest request) {}

  @Override
  public void setOdometryStdDevs(double xStd, double yStd, double rotStd) {}

  @Override
  public void addVisionEstimation(AprilCameraOutput cameraOutput) {}
}
