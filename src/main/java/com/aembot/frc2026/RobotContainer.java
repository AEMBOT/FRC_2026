// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.aembot.frc2026;

import com.aembot.frc2026.commands.CommandFactory;
import com.aembot.frc2026.subsystems.SubsystemFactory;
import com.aembot.lib.core.logging.Loggerable;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Loggerable {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final CommandXboxController secondaryController = new CommandXboxController(1);

  /* ---- DRIVETRAIN ---- */
  private final DriveSubsystem driveSubsystem = SubsystemFactory.createDriveSubsystem();

  /* ---- SHOOTER ---- */
  private final HoodSubsystem hoodSubsystem = SubsystemFactory.createHoodSubsystem();

  private final CommandFactory commandFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(LoggedRobot robot) {
    setupLogger(robot);
    configureBindings();

    this.commandFactory = new CommandFactory(driveSubsystem, hoodSubsystem);
  }

  /** Use this method to define your controller button -> command mappings */
  private void configureBindings() {
    driveSubsystem.setDefaultCommand(commandFactory.createDriveJoystickCmd(driverController));
    hoodSubsystem.setDefaultCommand(commandFactory.createHoodStopCommand());

    driverController.a().whileTrue(commandFactory.createHoodUpCommand());
    driverController.b().whileTrue(commandFactory.createHoodDownCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
