package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;

public final class DriveCommands {
  public static JoystickDriveCommand createDriveJoystickCmd(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {
    return JoystickDriveCommand.createCommandWithSteer(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        driverJoystick::getRightX,
        slowModeActiveSupplier);
  }
}
