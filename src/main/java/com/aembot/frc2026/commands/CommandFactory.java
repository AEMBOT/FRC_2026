package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  public final IntakeCommands intakeCommands;

  public CommandFactory(
      DriveSubsystem driveSubsystem,
      HoodSubsystem hoodSubsystem,
      OverBumperIntakeDeploySubsystem intakeDeploySubsystem,
      OverBumperIntakeRollerSubsystem intakeRollerSubsystem,
      FlywheelSubsystem flywheelSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;

    this.intakeCommands = new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem);
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

  public Command createFlywheelSlowSpinCommand() {
    return flywheelSubsystem.smartVelocitySetpointCommand(() -> 10);
  }

  public Command createFlywheelFastSpinCommand() {
    return flywheelSubsystem.smartVelocitySetpointCommand(() -> 20);
  }
}
