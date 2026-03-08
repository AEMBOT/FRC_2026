package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerSubsystem;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem;
import com.aembot.frc2026.subsystems.spindexer.SpindexerSubsystem;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public final class CommandFactory {

  private final DriveSubsystem driveSubsystem;
  public final IntakeCommands intakeCommands;
  public final IndexerCommands indexerCommands;
  public final ShooterCommands shooterCommands;

  private boolean shootFuel;
  private final Trigger aimTrigger;
  private final Trigger kickerTrigger;

  public CommandFactory(
      DriveSubsystem driveSubsystem,
      HoodSubsystem hoodSubsystem,
      OverBumperIntakeDeploySubsystem intakeDeploySubsystem,
      OverBumperIntakeRollerSubsystem intakeRollerSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      IndexerSelectorSubsystem indexerSelectorSubsystem,
      IndexerKickerSubsystem indexerKickerSubsystem,
      FlywheelSubsystem flywheelSubsystem,
      TurretSubsystem turretSubsystem) {

    this.driveSubsystem = driveSubsystem;
    this.intakeCommands = new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem);
    this.indexerCommands =
        new IndexerCommands(spindexerSubsystem, indexerSelectorSubsystem, indexerKickerSubsystem);
    this.shooterCommands = new ShooterCommands(hoodSubsystem, turretSubsystem, flywheelSubsystem);

    this.aimTrigger =
        new Trigger(() -> shootFuel && DriverStation.isAutonomousEnabled()).whileTrue(shooterCommands.createShootFuelCommand());
    this.kickerTrigger =
        new Trigger(() -> (shootFuel && shooterCommands.isShooterNearGoal() && DriverStation.isAutonomousEnabled()))
            .whileTrue(indexerCommands.createFeedIndexerCommand());
  }

  public void logCommands() {
    Logger.recordOutput("Commands/shootFuel", shootFuel);
    Logger.recordOutput("Commands/atSetpoint", shooterCommands.isShooterNearGoal());
  }

  public Command createShootFuelCommand() {
    return new RunCommand(() -> shootFuel = true).finallyDo(() -> shootFuel = false);
  }

  public Command createStartShootingFuelCommand() {
    return new InstantCommand(() -> shootFuel = true);
  }

  public Command createStopShootingFuelCommand() {
    return new InstantCommand(() -> shootFuel = false);
  }

  public Command createShootFuelTowerPosCommand() {
    return new ParallelCommandGroup(
        indexerCommands.createFeedIndexerCommand(),
        shooterCommands.createShootFuelCommand(),
        shooterCommands.createSetPoseSupplierToTowerCommand());
  }

  public JoystickDriveCommand createDriveJoystickCmd(
      CommandXboxController driverController, Trigger slowModeButton) {
    return DriveCommands.createDriveJoystickCmd(
        driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean());
  }

  public Command resetOdometryHeading() {
    return new InstantCommand(
        () -> {
          Translation2d robotTranslation =
              RobotStateYearly.get().getLatestFieldRobotPose().getTranslation();
          Rotation2d robotRotation =
              RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
          driveSubsystem.resetPose(new Pose2d(robotTranslation, robotRotation));
        });
  }

  public Command createSetDriveHeadingForUnderTrenchCommand(
      CommandXboxController driverController, Trigger slowModeButton) {
    BooleanSupplier inAllianceZone =
        () ->
            RobotRuntimeConstants.isBlueAlliance()
                ? RobotStateYearly.get().getLatestFieldRobotPose().getX() < 4.02844
                : RobotStateYearly.get().getLatestFieldRobotPose().getX() > 12.512548;

    return Commands.either(
        DriveCommands.createDriveWithForwardHeadingCommand(
            driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean()),
        DriveCommands.createDriveWithBackwardHeadingCommand(
            driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean()),
        inAllianceZone);
  }
}
