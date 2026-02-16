package com.aembot.frc2026.commands;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final TurretSubsystem turretSubsystem;

  public final IntakeCommands intakeCommands;

  public CommandFactory(
      DriveSubsystem driveSubsystem,
      HoodSubsystem hoodSubsystem,
      OverBumperIntakeDeploySubsystem intakeDeploySubsystem,
      OverBumperIntakeRollerSubsystem intakeRollerSubsystem,
      TurretSubsystem turretSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;

    this.intakeCommands = new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  public JoystickDriveCommand createDriveJoystickCmd(CommandXboxController driverController) {
    return DriveCommands.createDriveJoystickCmd(driveSubsystem, driverController.getHID());
  }

  public Command createHoodStopCommand() {
    return hoodSubsystem.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createHoodUpCommand() {
    return hoodSubsystem.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createHoodDownCommand() {
    return hoodSubsystem.smartVelocitySetpointCommand(() -> -30);
  }

  public Command createTurretStopCommand() {
    return turretSubsystem.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createTurretUpCommand() {
    return turretSubsystem.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createTurretDownCommand() {
    return turretSubsystem.smartVelocitySetpointCommand(() -> -30);
  }

}
