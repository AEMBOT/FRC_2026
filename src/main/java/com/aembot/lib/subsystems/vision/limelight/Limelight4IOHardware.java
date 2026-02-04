package com.aembot.lib.subsystems.vision.limelight;

import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.subsystems.vision.VisionInputs;
import com.aembot.lib.subsystems.vision.util.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;

public class Limelight4IOHardware implements LimelightIO {
  private final String kName;

  protected final CameraConfiguration kCameraConfiguration;
  protected final YearFieldConstantable kFieldConstants;

  protected final Supplier<Rotation2d> kRobotRotationSupplier;
  protected final Supplier<Double> kRobotYawVelocitySupplier;

  public Limelight4IOHardware(
      CameraConfiguration config,
      YearFieldConstantable fieldConstants,
      Supplier<Rotation2d> robotRotationSupplier,
      Supplier<Double> robotYawVelocitySupplier) {
    this.kCameraConfiguration = config;
    this.kFieldConstants = fieldConstants;
    this.kRobotRotationSupplier = robotRotationSupplier;
    this.kRobotYawVelocitySupplier = robotYawVelocitySupplier;

    this.kName = "limelight" + config.toString();

    updateMegatag2Info();
  }

  private void updateMegatag2Info() {
    LimelightHelpers.setCameraPose_RobotSpace(
        kName,
        kCameraConfiguration.CameraPose.getX(),
        kCameraConfiguration.CameraPose.getY(),
        kCameraConfiguration.CameraPose.getZ(),
        Units.radiansToDegrees(kCameraConfiguration.CameraPose.getRotation().getX()), // Roll
        Units.radiansToDegrees(kCameraConfiguration.CameraPose.getRotation().getY()), // Pitch
        Units.radiansToDegrees(kCameraConfiguration.CameraPose.getRotation().getZ()) // Yaw
        );

    Rotation2d robotRotation =
        kRobotRotationSupplier
            .get()
            .rotateBy(
                new Rotation2d(
                    kCameraConfiguration.MechanismPoseSupplier.get().getRotation().getZ()));
    double robotAngularVelocity =
        Units.radiansToDegrees(
            kRobotRotationSupplier.get().getRadians()
                + kCameraConfiguration.MechanismAngVelSupplier.get());

    LimelightHelpers.SetRobotOrientation(
        kName, robotRotation.getDegrees(), robotAngularVelocity, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {

    inputs.hasTag = hasTag();
    inputs.primaryTagID = getPrimaryTagID();
    inputs.estimatedRobotPose = getEstimatedPose();

    updateMegatag2Info();
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return this.kCameraConfiguration;
  }

  @Override
  public boolean hasTag() {
    return LimelightHelpers.getTV(kName);
  }

  @Override
  public int getPrimaryTagID() {
    // For some reason the getFiducialID method is setup to return a double,
    // despite docs saying the tid network table entry returns an int
    return (int) LimelightHelpers.getFiducialID(kName);
  }

  @Override
  public Pose2d getEstimatedPose() {

    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kName).pose;
  }

  @Override
  public void setThrottle(int throttle) {
    LimelightHelpers.SetThrottle(kName, throttle);
  }
}
