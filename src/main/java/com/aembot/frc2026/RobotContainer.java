// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.aembot.frc2026;

import com.aembot.frc2026.commands.CommandFactory;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.SubsystemFactory;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.util.AutoHelper;
import com.aembot.lib.core.logging.Loggerable;
import com.aembot.lib.subsystems.aprilvision.AprilVisionSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.generic.multimotor.IntakeRollerMultiMotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Loggerable {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);

  @SuppressWarnings("unused")
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  /* ---- FLYWHEEL ---- */
  private final FlywheelSubsystem flywheelSubsystem = SubsystemFactory.createFlywheelSubsystem();

  /* ---- DRIVETRAIN ---- */
  private final DriveSubsystem driveSubsystem = SubsystemFactory.createDriveSubsystem();

  /* ---- SHOOTER ---- */
  private final HoodSubsystem hoodSubsystem = SubsystemFactory.createHoodSubsystem();

  /* ---- INTAKE ---- */
  private final OverBumperIntakeDeploySubsystem intakeDeploySubsystem =
      SubsystemFactory.createIntakeDeploySubsystem();
  private final IntakeRollerMultiMotorSubsystem intakeRollerSubsystem =
      SubsystemFactory.createIntakeRollerSubsystem();

  /* ---- TURRET ---- */
  private final TurretSubsystem turretSubsystem = SubsystemFactory.createTurretSubsystem();

  private final CommandFactory commandFactory;

  /* ---- VISION ---- */
  private final AprilVisionSubsystem visionSubsystem =
      SubsystemFactory.createAprilVisionSubsystem();

  private final Trigger robotEnabled = new Trigger(() -> DriverStation.isEnabled());

  private final Trigger allianceInitialized =
      new Trigger(() -> DriverStation.getAlliance().isPresent());

  private final Trigger allianceIsRed =
      new Trigger(
          () ->
              allianceInitialized.getAsBoolean()
                  && DriverStation.getAlliance().get().equals(Alliance.Red));

  private final Trigger allianceIsBlue =
      new Trigger(
          () ->
              allianceInitialized.getAsBoolean()
                  && DriverStation.getAlliance().get().equals(Alliance.Blue));

  private final Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(LoggedRobot robot) {
    setupLogger(robot);

    this.commandFactory =
        new CommandFactory(
            driveSubsystem,
            hoodSubsystem,
            intakeDeploySubsystem,
            intakeRollerSubsystem,
            flywheelSubsystem,
            turretSubsystem);

    configureBindings();

    driveSubsystem.resetPose(new Pose2d(2, 4, Rotation2d.fromDegrees(-180)));
  }

  /** Use this method to define your controller button -> command mappings */
  private void configureBindings() {
    setupAutos().schedule();

    /* ---- DEFAULT COMMANDS ---- */

    // Use left bumper for slow mode
    driveSubsystem.setDefaultCommand(
        commandFactory.createDriveJoystickCmd(driverController, driverController.leftBumper()));

    hoodSubsystem.setDefaultCommand(commandFactory.shooterCommands.createHoodDownCommand());

    turretSubsystem.setDefaultCommand(
        commandFactory.shooterCommands.createTurretTowardsGoalCommand());

    intakeRollerSubsystem.setDefaultCommand(
        commandFactory.intakeCommands.createStopIntakeCommand());

    flywheelSubsystem.setDefaultCommand(
        commandFactory.shooterCommands.createFlywheelIdleSpeedCommand());

    /* ---- PRIMARY DRIVER COMMANDS ---- */

    driverController
        .rightTrigger()
        .whileTrue(
            commandFactory
                .createShootFuelCommand()
                .alongWith(commandFactory.intakeCommands.createRunIntakeCommand()));

    driverController.rightBumper().whileTrue(commandFactory.createShootFuelTowerPosCommand());

    // c on the controller
    driverController.leftStick().onTrue(commandFactory.intakeCommands.createZeroDownCommand());

    // z on the controller
    driverController.rightStick().onTrue(commandFactory.intakeCommands.createUpCommand());

    driverController
        .x()
        .whileTrue(
            commandFactory.createSetDriveHeadingForUnderTrenchCommand(
                driverController, driverController.leftBumper()));

    driverController
        .povLeft()
        .onTrue(commandFactory.shooterCommands.createSetPassingPoseLeftCommand());

    driverController
        .povUp()
        .onTrue(commandFactory.shooterCommands.createSetPassingPoseMiddleCommand());

    driverController
        .povRight()
        .onTrue(commandFactory.shooterCommands.createSetPassingPoseRightCommand());

    driverController
        .povDown()
        .onTrue(commandFactory.shooterCommands.createSetPassingPoseOutpostCommand());

    driverController.start().onTrue(commandFactory.resetOdometryHeading());

    /* ---- SECONDARY CONTROLLER BINDINGS ---- */

    secondaryController.leftBumper().onTrue(visionSubsystem.createKillVisionCommand());

    // rest is unused

    /* ---- ADDITIONAL TRIGGER BINDINGS ---- */

    robotEnabled
        .onTrue(visionSubsystem.updateNTEnabledCommand())
        .onFalse(visionSubsystem.updateNTDisabledCommand());

    // allianceIsBlue.onChange(setupAutos());
    // allianceIsRed.onChange(setupAutos());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoHelper.autoChooser.selectedCommandScheduler();
  }

  /**
   * Use this to pass the teleop init command to the main {@link Robot} class
   *
   * @return the command to run at the start of teleop
   */
  public Command getTeleopInitCommand() {
    return new ParallelCommandGroup(
        commandFactory.createStopShootingFuelCommand(),
        commandFactory.intakeCommands.createStopIntakeCommand());
  }

  public void logCommands() {
    commandFactory.logCommands();
    if (DriverStation.getAlliance().isPresent())
      Logger.recordOutput("Alliance", DriverStation.getAlliance().get());
    Logger.recordOutput("AllianceSet", DriverStation.getAlliance().isPresent());

    field.setRobotPose(RobotStateYearly.get().getLatestFieldRobotPose());
    SmartDashboard.putData("FieldData/Field2d", field);
  }

  private Command setupAutos() {
    return new InstantCommand(
            () -> {
              AutoHelper.setupAutoFactory(driveSubsystem);

              AutoHelper.registerAutoCommands(commandFactory);

              AutoHelper.setupAutoChooser();

              SmartDashboard.putData("Choose Auto Routine", AutoHelper.autoChooser);

              System.out.println("Setup Autos");
            })
        .ignoringDisable(true);
  }
}
