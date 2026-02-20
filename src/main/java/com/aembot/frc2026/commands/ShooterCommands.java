package com.aembot.frc2026.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final OptimalVelocityTable passingOutpostTable;
  private final OptimalVelocityTable passingLeftTable;
  private final OptimalVelocityTable passingMiddleTable;
  private final OptimalVelocityTable passingRightTable;

  private Supplier<OptimalVelocityTable> passingTableSupplier;

  public ShooterCommands(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel) {
    this.hood = hood;
    this.turret = turret;
    this.flywheel = flywheel;

    String velocityTableDirectory = Filesystem.getDeployDirectory() + "/initial-velocities/real/";
    if (RobotRuntimeConstants.MODE == RuntimeMode.SIM) {
      velocityTableDirectory = Filesystem.getDeployDirectory() + "/initial-velocities/sim/";
    }

    // Override shooting hub table to always use real trajectories
    this.shootingHubTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "../real/Shooting_Hub_Initial_Velocities.csv");
    this.passingOutpostTable =
        new OptimalVelocityTable(velocityTableDirectory + "Passing_Outpost_Initial_Velocities.csv");
    this.passingLeftTable =
        new OptimalVelocityTable(velocityTableDirectory + "Passing_Left_Initial_Velocities.csv");
    this.passingMiddleTable =
        new OptimalVelocityTable(velocityTableDirectory + "Passing_Middle_Initial_Velocities.csv");
    this.passingRightTable =
        new OptimalVelocityTable(velocityTableDirectory + "Passing_Right_Initial_Velocities.csv");
    passingTableSupplier = () -> passingMiddleTable;
  }

  /* ---- VELOCITY TABLES ---- */

  /**
   * @return
   */
  private OptimalVelocityTable getCurrentVelocityTable() {
    // TODO: get actual point to switch to passing
    if (RobotStateYearly.get().getLatestFieldRobotPose().getX() < 4) {
      return shootingHubTable;
    } else {
      return passingTableSupplier.get();
    }
  }

  /**
   * @return
   */
  public Command createSetPassingPoseOutpostCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingOutpostTable;
        });
  }

  /**
   * @return
   */
  public Command createSetPassingPoseLeftCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingLeftTable;
        });
  }

  /**
   * @return
   */
  public Command createSetPassingPoseMiddleCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingMiddleTable;
        });
  }

  /**
   * @return
   */
  public Command createSetPassingPoseRightCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingRightTable;
        });
  }

  /**
   * @return
   */
  private Rotation2d getCurrentYaw() {
    return getCurrentVelocityTable()
        .getFuelInitVelocityRotation3d(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds())
        .toRotation2d();
  }

  /**
   * @return
   */
  private double getCurrentPitch() {
    return Units.radiansToDegrees(
        getCurrentVelocityTable()
            .getFuelInitVelocityRotation3d(
                RobotStateYearly.get().getLatestFieldRobotPose(),
                RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds())
            .getY());
  }

  /**
   * @return
   */
  private double getCurrentSpeed() {
    return getCurrentVelocityTable()
        .getFuelInitVelocityMagnitude(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
  }

  /* ---- HOOD COMMANDS ---- */

  /**
   * @return
   */
  public Command createHoodTowardsHubCommand() {
    return new RepeatCommand(hood.smartPositionSetpointCommand(() -> getCurrentPitch()));
  }

  /* ---- TURRET COMMANDS ---- */

  /**
   * @return
   */
  private double getTurretTowardsHubFromRobotPose() {
    double targetRotation =
        getCurrentYaw()
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

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * <p>TODO: find better value than 10
   *
   * @return True if turret is within 10 degrees of goal position, false otherwise
   */
  public boolean isTurretNearGoal() {
    return MathUtil.isNear(turret.getCurrentPosition(), getTurretTowardsHubFromRobotPose(), 10);
  }

  /* ---- FLYWHEEL COMMANDS ---- */

  public Command createFlywheelHubSpeedCommand() {
    return new RepeatCommand(flywheel.smartVelocitySetpointCommand(() -> getCurrentSpeed()));
  }

  /* ---- FUEL SHOOTING FUNCTIONS ---- */

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

  /** Create a RebuiltFuelOnFly based off of the current state of all of the subsystems */
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
                    MetersPerSecond.of(flywheel.getCurrentVelocity()),
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
