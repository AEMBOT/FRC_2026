package com.aembot.frc2026.commands;

import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;

  public final ShooterCommands shooterCommands;
  public final IntakeCommands intakeCommands;

  public CommandFactory(
      DriveSubsystem driveSubsystem,
      HoodSubsystem hoodSubsystem,
      OverBumperIntakeDeploySubsystem intakeDeploySubsystem,
      OverBumperIntakeRollerSubsystem intakeRollerSubsystem,
      TurretSubsystem turretSubsystem) {
    this.driveSubsystem = driveSubsystem;

    this.intakeCommands = new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem);
    this.shooterCommands = new ShooterCommands(hoodSubsystem, turretSubsystem);
  }

  public JoystickDriveCommand createDriveJoystickCmd(CommandXboxController driverController) {
    return DriveCommands.createDriveJoystickCmd(driveSubsystem, driverController.getHID());
  }
}
