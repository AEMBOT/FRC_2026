package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.aembot.lib.tracing.Traced;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

/** No-op implementation of {@link DrivetrainIO} for replay */
public class DrivetrainIOReplay implements DrivetrainIO {
  @Override
  @Traced
  public void updateInputs(DrivetrainInputs inputs) {}

  @Override
  @Traced
  public void logModules(DrivetrainInputs inputs, String prefix) {}

  @Override
  @Traced
  public void resetOdometry(Pose2d pose) {}

  @Override
  @Traced
  public void setRequest(SwerveRequest request) {}

  @Override
  @Traced
  public void setOdometryStdDevs(double xStd, double yStd, double rotStd) {}

  @Override
  @Traced
  public void addVisionEstimation(AprilCameraOutput cameraOutput) {}
}
