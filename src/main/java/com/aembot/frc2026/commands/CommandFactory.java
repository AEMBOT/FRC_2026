package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class CommandFactory {
  private final DriveSubsystem driveSubsystem;

  public CommandFactory(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public JoystickDriveCommand createDriveJoystickCmd(CommandXboxController driverController) {
    return DriveCommands.createDriveJoystickCmd(driveSubsystem, driverController.getHID());
  }
}
