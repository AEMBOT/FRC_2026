package com.aembot.lib.core.phoenix6;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

/**
 * Plain-Old-Data class holding the state of the swerve drivetrain. This encapsulates most data that
 * is relevant for telemetry or decision-making from the Swerve Drive.
 *
 * <p>AEMSwerveDriveState can also hold a timestamp synchronized with the RIO clock
 */
public class AEMSwerveDriveState extends SwerveDriveState {
  /**
   * {@link SwerveDriveState#Timestamp} but synchronized to the RIO clock instead of the CTRE one
   */
  public double timestampRIOSynchronized = Double.NaN;

  @Override
  public AEMSwerveDriveState clone() {
    // fromSwerveDriveState is mostly a copy-paste of super.clone()
    AEMSwerveDriveState toReturn = fromSwerveDriveState(this);
    toReturn.timestampRIOSynchronized = this.timestampRIOSynchronized;
    return toReturn;
  }

  public static AEMSwerveDriveState fromSwerveDriveState(SwerveDriveState swerveDriveState) {
    final AEMSwerveDriveState toReturn = new AEMSwerveDriveState();
    toReturn.Pose = swerveDriveState.Pose;
    toReturn.Speeds = swerveDriveState.Speeds;
    toReturn.ModuleStates = swerveDriveState.ModuleStates.clone();
    toReturn.ModuleTargets = swerveDriveState.ModuleTargets.clone();
    toReturn.ModulePositions = swerveDriveState.ModulePositions.clone();
    toReturn.RawHeading = swerveDriveState.RawHeading;
    toReturn.Timestamp = swerveDriveState.Timestamp;
    toReturn.OdometryPeriod = swerveDriveState.OdometryPeriod;
    toReturn.SuccessfulDaqs = swerveDriveState.SuccessfulDaqs;
    toReturn.FailedDaqs = swerveDriveState.FailedDaqs;
    return toReturn;
  }
}
