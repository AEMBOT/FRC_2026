package com.aembot.frc2026.commands;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class CommandFactory {

  private final DriveSubsystem driveSubsystem;
  public final IntakeCommands intakeCommands;
  public final IndexerCommands indexerCommands;
  public final ShooterCommands shooterCommands;

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
  }

  public Command createShootFuelCommand() {
    return new ParallelCommandGroup(
        indexerCommands.createFeedIndexerCommand(), shooterCommands.createShootFuelCommand());
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
}
