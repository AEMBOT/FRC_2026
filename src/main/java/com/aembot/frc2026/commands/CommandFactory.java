package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final HoodSubsystem hoodSubsystem;

  public CommandFactory(DriveSubsystem driveSubsystem, HoodSubsystem hoodSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
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
}
