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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  public final ShooterCommands shooterCommands;
  public final IntakeCommands intakeCommands;
  public final IndexerCommands indexerCommands;

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

    this.hoodSubsystem = hoodSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;

    this.intakeCommands = new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem);
    this.indexerCommands =
        new IndexerCommands(spindexerSubsystem, indexerSelectorSubsystem, indexerKickerSubsystem);
    this.driveSubsystem = driveSubsystem;

    this.shooterCommands = new ShooterCommands(hoodSubsystem, turretSubsystem, flywheelSubsystem);
  }

  public JoystickDriveCommand createDriveJoystickCmd(CommandXboxController driverController) {
    return DriveCommands.createDriveJoystickCmd(driveSubsystem, driverController.getHID());
  }
}
