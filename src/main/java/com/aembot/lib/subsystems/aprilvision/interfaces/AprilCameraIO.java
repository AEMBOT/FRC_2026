package com.aembot.lib.subsystems.aprilvision.interfaces;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface AprilCameraIO {
  public CameraConfiguration getConfiguration();

  public void updateInputs(AprilVisionInputs inputs);

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
