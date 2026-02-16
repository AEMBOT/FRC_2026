package com.aembot.frc2026.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public final class ShooterCommandFactory {

  private final HoodSubsystem hood;

  private final OptimalVelocityTable shootingHubTable;

  public ShooterCommandFactory(HoodSubsystem hood) {
    this.hood = hood;

    String robotType;
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        robotType = "/sim";
        break;

      case REPLAY:

      case REAL:

      default:
        robotType = "/real";
        break;
    }
    ;

    this.shootingHubTable =
        new OptimalVelocityTable(
            Filesystem.getDeployDirectory()
                + "/initial-velocities"
                + robotType
                + "/Shooting_Hub_Initial_Velocities.csv");
  }

  public Command createHoodTowardsHubCommand() {
    Supplier<Rotation3d> rotationSupplier = () -> getShootingAngle();

    return new RepeatCommand(
        hood.smartPositionSetpointCommand(
            () -> Units.radiansToDegrees(rotationSupplier.get().getY())));
  }

  public Command createShootFuelCommand() {

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new InstantCommand(() -> shootSimulatedFuel(), hood);
      case REPLAY:

      case REAL:

      default:
        return new RunCommand(() -> {}); // TODO create real shoot command
    }
  }

  private Rotation3d getShootingAngle() {
    return shootingHubTable.getFuelInitVelocityRotation3d(
        RobotStateYearly.get().getLatestFieldRobotPose(),
        RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
  }

  private Rotation2d getRelativeYaw() {
    return shootingHubTable
        .getFuelInitVelocityRotation3d(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds())
        .toRotation2d();
  }

  private double getShootingSpeed() {
    return shootingHubTable.getFuelInitVelocityMagnitude(
        RobotStateYearly.get().getLatestFieldRobotPose(),
        RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
  }

  private void shootSimulatedFuel() {

    Pose2d robotPose = RobotStateYearly.get().getLatestFieldRobotPose();

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    new Translation2d(),
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds(),
                        robotPose.getRotation()),
                    getRelativeYaw(), // TODO: replace with turret angle
                    Meters.of(0.5), // TODO:find spot to replace magic number
                    MetersPerSecond.of(getShootingSpeed()), // TODO: replace with flywheel speed
                    Degrees.of(hood.getCurrentPosition()))
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) ->
                        Logger.recordOutput(
                            "FieldSimulation/successfulShotsTrajectory",
                            poses.toArray(Pose3d[]::new)),
                    (poses) ->
                        Logger.recordOutput(
                            "FieldSimulation/missedShotsTrajectory",
                            poses.toArray(Pose3d[]::new))));
  }
}
