package com.aembot.frc2026.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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

public final class ShooterCommands {

  private final HoodSubsystem hood;
  private final TurretSubsystem turret;
  private final FlywheelSubsystem flywheel;
  private final OptimalVelocityTable shootingHubTable;

  public ShooterCommands(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel) {
    this.hood = hood;
    this.turret = turret;

    String robotType;
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        robotType = "/real";
        break;

      case REPLAY:

      case REAL:

      default:
        robotType = "/real";
        break;
    }

    this.shootingHubTable =
        new OptimalVelocityTable(
            Filesystem.getDeployDirectory()
                + "/initial-velocities"
                + robotType
                + "/Shooting_Hub_Initial_Velocities.csv");
    this.flywheel = flywheel;
  }

  public Command createHoodStopCommand() {
    return hood.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createHoodUpCommand() {
    return hood.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createHoodDownCommand() {
    return hood.smartVelocitySetpointCommand(() -> -30);
  }

  public Command createTurretStopCommand() {
    return turret.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createTurretLeftCommand() {
    return turret.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createTurretRightCommand() {
    return turret.smartVelocitySetpointCommand(() -> -30);
  }

  public Command createFlywheelSlowSpinCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> 10);
  }

  public Command createFlywheelFastSpinCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> 20);
  }

  public Command createHoodTowardsHubCommand() {
    Supplier<Rotation3d> rotationSupplier = () -> getShootingAngle();

    return new RepeatCommand(
        hood.smartPositionSetpointCommand(
            () -> Units.radiansToDegrees(rotationSupplier.get().getY())));
  }

  private double getTurretTowardsHubFromRobotPose() {
    double targetRotation =
        getRelativeYaw()
                .minus(RobotStateYearly.get().getLatestFieldRobotPose().getRotation())
                .getDegrees()
            + 180;

    if (targetRotation < 0) {
      targetRotation += 360;
    }

    return targetRotation;
  }

  public Command createTurretTowardsHubCommand() {
    return new RepeatCommand(
        turret.smartPositionSetpointCommand(() -> getTurretTowardsHubFromRobotPose()));
  }

  public Command createShootFuelCommand() {

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new InstantCommand(() -> shootSimulatedFuel());
      case REPLAY:

      case REAL:

      default:
        return new RunCommand(() -> {}); // TODO create real shoot command
    }
  }

  private Rotation3d getShootingAngle() {
    Rotation3d rot =
        shootingHubTable.getFuelInitVelocityRotation3d(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());

    Logger.recordOutput("TEST", Units.radiansToDegrees(rot.getY()));

    return rot;
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
                    RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds(),
                    new Rotation2d(Units.degreesToRadians(turret.getCurrentPosition() + 180))
                        .plus(robotPose.getRotation()),
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
