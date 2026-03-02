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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        new Trigger(() -> shootFuel).whileTrue(shooterCommands.createShootFuelCommand());
    this.kickerTrigger =
        new Trigger(() -> (shootFuel && shooterCommands.isShooterNearGoal()))
            .whileTrue(indexerCommands.createFeedIndexerCommand());
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

  public JoystickDriveCommand createDriveJoystickCmd(CommandXboxController driverController) {
    return DriveCommands.createDriveJoystickCmd(driveSubsystem, driverController.getHID());
  }
}
