package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainHardwareIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainIOReplay;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Subsystem factories intended to be called from RobotContainer */
public class SubsystemFactory {
  public static DriveSubsystem createDriveSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new DriveSubsystem(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                new DrivetrainSimIO(
                    RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedDrivetrainConfiguration(),
                    RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                    RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations()))
            .withSetPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));
      case REPLAY:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainIOReplay());
      case REAL:
      default:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations()));
    }
  }
}
