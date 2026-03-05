package com.aembot.lib.subsystems.aprilvision.interfaces;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

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

  /**
   * Adjust the given standard deviations to take into account the distance between the robot pose
   * estimated by the whole odometry system and the pose estimated by this camera
   *
   * @param unadjustedStandardDevs The standard deviations not taking into account the above
   * @param wholeEstimatedRobotPose The estimated robot pose from robot state.
   * @param cameraEstimatedRobotPose The estimated robot pose from this camera
   * @return Adjusted std devs
   */
  default OdometryStandardDevs adjustStdDevsWithOdomPose(
      OdometryStandardDevs unadjustedStandardDevs,
      Pose2d wholeEstimatedRobotPose,
      Pose2d cameraEstimatedRobotPose) {

    // // Guard against null or NaN poses
    // if (wholeEstimatedRobotPose == null
    //     || cameraEstimatedRobotPose == null
    //     || Double.isNaN(wholeEstimatedRobotPose.getX())
    //     || Double.isNaN(wholeEstimatedRobotPose.getY())
    //     || Double.isNaN(cameraEstimatedRobotPose.getX())
    //     || Double.isNaN(cameraEstimatedRobotPose.getY())) {

    //   Logger.recordOutput(getConfiguration() + "/stdDevs", unadjustedStandardDevs);
    //   return unadjustedStandardDevs;
    // }

    double distMeters =
        wholeEstimatedRobotPose.minus(cameraEstimatedRobotPose).getTranslation().getNorm();
    double factor = 1 + (Math.pow(distMeters, 2) * 2); // Prolly very subject to change

    Logger.recordOutput(
        getConfiguration() + "/stdDevs",
        new OdometryStandardDevs(
            unadjustedStandardDevs.xStdDev() * factor,
            unadjustedStandardDevs.yStdDev() * factor,
            unadjustedStandardDevs.rotStdDev() * factor));
    return new OdometryStandardDevs(
        unadjustedStandardDevs.xStdDev() * factor,
        unadjustedStandardDevs.yStdDev() * factor,
        unadjustedStandardDevs.rotStdDev() * factor);
  }

  public void throttleForDisabled();

  public void throttleForEnabled();
}
