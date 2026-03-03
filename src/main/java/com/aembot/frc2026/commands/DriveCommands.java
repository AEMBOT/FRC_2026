package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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

  public static JoystickDriveCommand createDriveWithForwardHeadingCommand(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {

    Supplier<Rotation2d> headingSupplier =
        () -> RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;

    return JoystickDriveCommand.createCommandWithHeading(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        headingSupplier,
        slowModeActiveSupplier);
  }

  public static JoystickDriveCommand createDriveWithBackwardHeadingCommand(
      DriveSubsystem subsystem,
      XboxController driverJoystick,
      BooleanSupplier slowModeActiveSupplier) {

    Supplier<Rotation2d> headingSupplier =
        () -> RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;

    return JoystickDriveCommand.createCommandWithHeading(
        subsystem,
        RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
        driverJoystick::getLeftY,
        driverJoystick::getLeftX,
        headingSupplier,
        slowModeActiveSupplier);
  }
}
