package com.aembot.frc2026.util;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoHelper {

  public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /**
   * Setip the auto builder
   *
   * @param driveSubsystem The drive subsystem to control
   */
  public static void setupAutoBuilder(DriveSubsystem driveSubsystem) {
    AutoBuilder.configure(
        () -> RobotStateYearly.get().getLatestFieldRobotPose(),
        (pose) -> driveSubsystem.resetPose(pose),
        () -> RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds(),
        (speeds, feedforwards) -> driveSubsystem.setRequestFromChassisSpeeds(speeds),
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration().autoController,
        RobotRuntimeConstants.getPathplannerConfig(),
        () -> RobotRuntimeConstants.isRedAlliance(),
        driveSubsystem);
  }

  /** Add all autos to the auto chooser DOES NOT add auto chooser to dashboard */
  public static void setupAutoChooser() {

    addAuto("New Auto");
  }

  /**
   * Add an auto to the auto chooser
   *
   * @param autoName name of the auto to add
   */
  private static void addAuto(String autoName) {
    autoChooser.addOption(autoName, new PathPlannerAuto(autoName));
  }
}
