package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.intake.generic.multimotor.IntakeRollerMultiMotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.leds.LEDStripSubsystem;
import com.aembot.lib.subsystems.leds.LEDStripSubsystem.LEDPattern;
import com.aembot.lib.subsystems.leds.LEDStripSubsystem.LEDSpeed;
import com.aembot.lib.subsystems.premades.BinaryVoltageMotorFollowerSubsytem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import java.util.function.BooleanSupplier;

public final class CommandFactory {

  private final DriveSubsystem driveSubsystem;
  public final IntakeCommands intakeCommands;
  public final ShooterCommands shooterCommands;

  private boolean shootFuel;
  private final Trigger aimTrigger;

  public CommandFactory(
      DriveSubsystem driveSubsystem,
      HoodSubsystem hoodSubsystem,
      OverBumperIntakeDeploySubsystem intakeDeploySubsystem,
      IntakeRollerMultiMotorSubsystem intakeRollerSubsystem,
      BinaryVoltageMotorFollowerSubsytem intakeWheelsSubsystem,
      FlywheelSubsystem flywheelSubsystem,
      TurretSubsystem turretSubsystem) {

    this.driveSubsystem = driveSubsystem;
    this.intakeCommands =
        new IntakeCommands(intakeDeploySubsystem, intakeRollerSubsystem, intakeWheelsSubsystem);
    this.shooterCommands = new ShooterCommands(hoodSubsystem, turretSubsystem, flywheelSubsystem);

    this.aimTrigger =
        new Trigger(() -> shootFuel)
            .whileTrue(
                shooterCommands
                    .createShootFuelCommand()
                    .alongWith(intakeRollerSubsystem.runRollerCommand())
                    .alongWith(intakeWheelsSubsystem.runSystemCommand()));
  }

  public void logCommands() {
    AEMLogger.recordOutput("Commands/shootFuel", shootFuel);
    AEMLogger.recordOutput("Commands/atSetpoint", shooterCommands.isShooterNearGoal());
  }

  public Command createShootFuelCommand() {
    return new RunCommand(() -> shootFuel = true).finallyDo(() -> shootFuel = false);
  }

  public Command createStartShootingFuelCommand() {
    return new InstantCommand(() -> shootFuel = true);
  }

  public Command createStopShootingFuelCommand() {
    return new InstantCommand(() -> shootFuel = false);
  }

  public Command createShootFuelTowerPosCommand() {
    return new ParallelCommandGroup(
        shooterCommands.createShootFuelCommand(),
        shooterCommands.createSetPoseSupplierToTowerCommand());
  }

  public JoystickDriveCommand createDriveJoystickCmd(
      CommandXboxController driverController, Trigger slowModeButton) {
    return DriveCommands.createDriveJoystickCmd(
        driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean());
  }

  public Command resetOdometryHeading() {
    return new InstantCommand(
        () -> {
          Translation2d robotTranslation =
              RobotStateYearly.get().getLatestFieldRobotPose().getTranslation();
          Rotation2d robotRotation =
              RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
          driveSubsystem.resetPose(new Pose2d(robotTranslation, robotRotation));
        });
  }

  public Command createSetDriveHeadingForUnderTrenchCommand(
      CommandXboxController driverController, Trigger slowModeButton) {
    BooleanSupplier inAllianceZone =
        () ->
            RobotRuntimeConstants.isBlueAlliance()
                ? RobotStateYearly.get().getLatestFieldRobotPose().getX() < 4.02844
                : RobotStateYearly.get().getLatestFieldRobotPose().getX() > 12.512548;

    return Commands.either(
        DriveCommands.createDriveWithForwardHeadingCommand(
            driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean()),
        DriveCommands.createDriveWithBackwardHeadingCommand(
            driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean()),
        inAllianceZone);
  }

  public void configureLEDStripTriggers(LEDStripSubsystem ledStripSubsystem) {
    ledStripSubsystem.setDefaultCommand(
        new RepeatCommand(
                new DeferredCommand(
                    () ->
                        ledStripSubsystem.patternAndSpeedCommand(
                            RobotRuntimeConstants.isRedAlliance()
                                ? LEDPattern.RED
                                : LEDPattern.BLUE,
                            LEDSpeed.NORMAL),
                    Set.of(ledStripSubsystem)))
            .ignoringDisable(true));

    this.aimTrigger.whileTrue(ledStripSubsystem.hueFlashCommand());
  }
}
