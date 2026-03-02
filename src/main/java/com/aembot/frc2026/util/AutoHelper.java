package com.aembot.frc2026.util;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.aembot.frc2026.commands.CommandFactory;
import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoHelper {

  public static final AutoChooser autoChooser = new AutoChooser();

  public static AutoFactory autoFactory;

  /**
   * Setip the auto builder
   *
   * @param driveSubsystem The drive subsystem to control
   */
  public static void setupAutoFactory(DriveSubsystem driveSubsystem) {

    autoFactory =
        new AutoFactory(
            () -> RobotStateYearly.get().getLatestFieldRobotPose(),
            (pose) -> driveSubsystem.resetPose(pose),
            (sample) ->
                driveSubsystem.setRequestFromSwerveSample(
                    (SwerveSample) sample, RobotStateYearly.get().getLatestFieldRobotPose()),
            RobotRuntimeConstants.isRedAlliance(),
            driveSubsystem);
  }

  /**
   * Add all autos to the auto chooser
   *
   * <p>DOES NOT add auto chooser to dashboard
   */
  public static void setupAutoChooser() {

    addAuto("EventMarkerTest");
  }

  /**
   * Add an auto to the auto chooser
   *
   * @param autoName name of the auto to add
   */
  private static void addAuto(String autoName) {

    AutoRoutine routine = autoFactory.newRoutine(autoName);

    AutoTrajectory traj = routine.trajectory(autoName);

    routine.active().onTrue(traj.cmd());

    autoChooser.addRoutine(autoName, () -> routine);
  }

  /**
   * Register all auto commands to use in choreo
   *
   * @param commandFactory the command factory used by robot container
   */
  public static void registerAutoCommands(CommandFactory commandFactory) {

    autoFactory
        .bind("Test", Commands.print("TRIGGERED"))
        .bind("DeployIntake", commandFactory.intakeCommands.createDownCommand())
        .bind("RaiseIntake", commandFactory.intakeCommands.createUpCommand())
        .bind("RunIntake", commandFactory.intakeCommands.createRunIntakeCommand())
        .bind("StopIntake", commandFactory.intakeCommands.createStopIntakeCommand());
  }
}
