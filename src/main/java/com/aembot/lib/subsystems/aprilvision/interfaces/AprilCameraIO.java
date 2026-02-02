package com.aembot.lib.subsystems.aprilvision.interfaces;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface AprilCameraIO {
  public CameraConfiguration getConfiguration();

  public void updateInputs(AprilVisionInputs inputs);

  /**
   * Get the 2d field-centric offset from the robot center to the camera, taking robot yaw into
   * account. For example, if the camera's offset from the drivetrain center is (1,0), and the
   * robot's yaw is 90 degrees counterclockwise, the field-centric offset will be (0,1).
   *
   * @param robotYaw The yaw of the robot on the field
   */
  default Translation2d computeCameraOffsetFieldCentric(Rotation2d robotYaw) {
    return getConfiguration()
        .getCameraPosition()
        .getTranslation()
        .toTranslation2d()
        .rotateBy(robotYaw);
  }

  /**
   * Compute the robot-centric translation from the robot to the tag
   *
   * @param robotYaw The yaw of the robot at time of computation
   * @param cameraRotationToTarget The rotation from the camera to the tag
   * @param distanceToTagMeters The distance to the tag in meters
   * @return Translation from the robot chassis' center to the tag
   */
  default Translation2d computeRobotToTag(
      Rotation2d cameraRotationToTarget, double distanceToTagMeters) {
    // Scale the camera rotation by the camera rotation ratio scalar set in the configuration,
    // invert the system because limelight and wpilib rotations are inverted.
    Rotation2d scaledCameraRotationToTarget =
        Rotation2d.fromDegrees(
            -(cameraRotationToTarget.getDegrees() / getConfiguration().cameraXRotationScalar));

    return new Translation2d(distanceToTagMeters, scaledCameraRotationToTarget)
        .plus(getConfiguration().getCameraPosition().getTranslation().toTranslation2d());
  }

  default VisionPoseEstimation computeRobotPose(
      Pose2d tagPose,
      Rotation2d robotRotation,
      Translation2d robotCenterToTag,
      ChassisSpeeds latestFieldChassisSpeeds,
      double latency) {
    Translation2d latencyUncompensatedPosition =
        tagPose.getTranslation().minus(robotCenterToTag.rotateBy(robotRotation));

    Translation2d latencyCompensatedPosition =
        compensateForEstimateLatency(
            latencyUncompensatedPosition, latestFieldChassisSpeeds, latency);

    return new VisionPoseEstimation(
        new Pose2d(latencyUncompensatedPosition, robotRotation),
        new Pose2d(latencyCompensatedPosition, robotRotation));
  }

  default Translation2d compensateForEstimateLatency(
      Translation2d uncompensatedPosition, ChassisSpeeds latestFieldChassisSpeeds, double latency) {
    return new Translation2d(
        uncompensatedPosition.getX() + (latestFieldChassisSpeeds.vxMetersPerSecond * latency),
        uncompensatedPosition.getY() + (latestFieldChassisSpeeds.vyMetersPerSecond * latency));
  }

  default Pose2d compensateForEstimateLatency(
      Pose2d uncompensatedPose, ChassisSpeeds latestFieldChassisSpeeds, double latency) {
    return new Pose2d(
        compensateForEstimateLatency(
            uncompensatedPose.getTranslation(), latestFieldChassisSpeeds, latency),
        uncompensatedPose
            .getRotation()
            .minus(
                Rotation2d.fromRadians(latestFieldChassisSpeeds.omegaRadiansPerSecond)
                    .times(latency)));
  }
}
