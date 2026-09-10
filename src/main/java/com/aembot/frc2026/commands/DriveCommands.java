package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.lib.subsystems.aprilvision.AprilVisionSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  public static final double HUB_STRAFE_TEST_DISTANCE_METERS = 3.0;

  /**
   * Start on the alliance-wall side of the hub with the turret near its 180-degree center. The
   * chassis field arrow points away from the hub under the current shooter convention.
   */
  public static Pose2d getHubStrafeTestPose(Alliance alliance) {
    double blueX = Field2026.BLUE_HUB_CENTER.getX() - HUB_STRAFE_TEST_DISTANCE_METERS;
    double blueY = Field2026.BLUE_HUB_CENTER.getY();
    if (alliance == Alliance.Red) {
      return new Pose2d(
          Field2026.get().getFieldLayout().getFieldLength() - blueX,
          Field2026.get().getFieldLayout().getFieldWidth() - blueY,
          Rotation2d.kZero);
    }
    return new Pose2d(blueX, blueY, Rotation2d.k180deg);
  }

  /** One-shot virtual field placement; subsequent strafing still uses live odometry. */
  public static Command createResetHubStrafeTestPoseCommand(
      DriveSubsystem drive, AprilVisionSubsystem vision) {
    return new InstantCommand(
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isEmpty()) {
                SmartDashboard.putString(
                    "Hub Strafe Test Status", "Select a DS alliance, then press reset again");
                return;
              }
              Pose2d pose = getHubStrafeTestPose(alliance.get());
              vision.setVisionEnabled(false);
              drive.resetPose(pose);
              Logger.recordOutput("HubStrafeTest/ResetPose", pose);
              Logger.recordOutput("HubStrafeTest/ResetTimestampSeconds", Timer.getFPGATimestamp());
              SmartDashboard.putString(
                  "Hub Strafe Test Status",
                  alliance.get() + " reset: hub 3 m away, turret centered, vision OFF");
            },
            drive)
        .ignoringDisable(true)
        .withName("Reset Hub Strafe Test Pose (Vision Off)");
  }

  public static JoystickDriveCommand createDriveJoystickCmd(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {
    return JoystickDriveCommand.createCommandWithSteer(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        driverJoystick::getRightX,
        slowModeActiveSupplier);
  }

  public static JoystickDriveCommand createDriveWithForwardHeadingCommand(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {

    Supplier<Rotation2d> headingSupplier =
        () -> RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;

    return JoystickDriveCommand.createCommandWithHeading(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        headingSupplier,
        slowModeActiveSupplier);
  }

  public static JoystickDriveCommand createDriveWithBackwardHeadingCommand(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {

    Supplier<Rotation2d> headingSupplier =
        () -> RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;

    return JoystickDriveCommand.createCommandWithHeading(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        headingSupplier,
        slowModeActiveSupplier);
  }
}
