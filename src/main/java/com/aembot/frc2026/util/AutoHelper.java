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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class AutoHelper {

  public static final AutoChooser autoChooser = new AutoChooser();

  public static AutoFactory autoFactory;

  public static Consumer<Pose2d> setOdometryFunc;

  /**
   * Setip the auto builder
   *
   * @param driveSubsystem The drive subsystem to control
   */
  public static void setupAutoFactory(DriveSubsystem driveSubsystem) {

    setOdometryFunc = (pose) -> driveSubsystem.resetPose(pose);

    autoFactory =
        new AutoFactory(
            () -> RobotStateYearly.get().getLatestFieldRobotPose(),
            (pose) -> driveSubsystem.resetPose(pose),
            (SwerveSample sample) -> driveSubsystem.setRequestFromSwerveSample(sample),
            RobotRuntimeConstants.isRedAlliance(),
            driveSubsystem,
            (state, isStart) -> Logger.recordOutput("AUTO_TRAJ", state.getPoses()));
  }

  /**
   * Add all autos to the auto chooser
   *
   * <p>DOES NOT add auto chooser to dashboard
   */
  public static void setupAutoChooser() {

    addAuto("MiddleDepot");
    addAuto("LeftNeutralDepot");
    addAuto("RightNeutralOutpost");
    addAuto("TowerPreload");

    // TODO make clean
    AutoRoutine doNothingRoutine = autoFactory.newRoutine("DoNothing");

    doNothingRoutine
        .active()
        .onTrue(
            new InstantCommand(
                () ->
                    setOdometryFunc.accept(
                        new Pose2d(
                            0,
                            0,
                            DriverStation.getAlliance().get() == Alliance.Blue
                                ? new Rotation2d()
                                : Rotation2d.k180deg))));

    autoChooser.addRoutine("DoNothing", () -> doNothingRoutine);
  }

  /**
   * Add an auto to the auto chooser
   *
   * @param autoName name of the auto to add
   */
  private static void addAuto(String autoName) {

    AutoRoutine routine = autoFactory.newRoutine(autoName);

    AutoTrajectory traj = routine.trajectory(autoName);

    routine
        .active()
        .onTrue(
            new InstantCommand(() -> setOdometryFunc.accept(traj.getInitialPose().orElseThrow()))
                .andThen(traj.cmd()));

    autoChooser.addRoutine(autoName, () -> routine);
  }

  /**
   * Register all auto commands to use in choreo
   *
   * @param commandFactory the command factory used by robot container
   */
  public static void registerAutoCommands(CommandFactory commandFactory) {

    autoFactory
        .bind("DeployIntake", commandFactory.intakeCommands.createDownCommand())
        .bind("RaiseIntake", commandFactory.intakeCommands.createUpCommand())
        .bind("RunIntake", commandFactory.intakeCommands.createRunIntakeCommand())
        .bind("StopIntake", commandFactory.intakeCommands.createStopIntakeCommand())
        .bind("StartShooting", commandFactory.createStartShootingFuelCommand())
        .bind("StopShooting", commandFactory.createStopShootingFuelCommand())
        .bind(
            "StartFlickingIntake",
            commandFactory.intakeCommands.createContinuousFlickIntakeCommand())
        .bind("StopFlickingIntake", commandFactory.intakeCommands.createDownCommand());
  }
}
