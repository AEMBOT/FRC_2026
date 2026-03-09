package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.aembot.lib.tracing.Traced;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

/** No-op implementation of {@link DrivetrainIO} for replay */
public class DrivetrainIOReplay implements DrivetrainIO {
  @Override
  @Traced(category = "Drivetrain")
  public void updateInputs(DrivetrainInputs inputs) {}

  @Override
  @Traced(category = "Drivetrain")
  public void logModules(DrivetrainInputs inputs, String prefix) {}

  @Override
  @Traced(category = "Drivetrain")
  public void resetOdometry(Pose2d pose) {}

  @Override
  @Traced(category = "Drivetrain")
  public void setRequest(SwerveRequest request) {}

  @Override
  @Traced(category = "Drivetrain")
  public void setOdometryStdDevs(double xStd, double yStd, double rotStd) {}

  @Override
  @Traced(category = "Drivetrain")
  public void addVisionEstimation(AprilCameraOutput cameraOutput) {}
}
