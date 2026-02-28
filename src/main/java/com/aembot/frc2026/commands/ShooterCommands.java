package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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

  private final BooleanSupplier inShootingZone;

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

    // Supplier so that our shooting zones are different whether we are blue or red
    inShootingZone =
        RobotRuntimeConstants.isBlueAlliance()
            ? () -> RobotStateYearly.get().getLatestFieldRobotPose().getX() < 4.02844
            : () -> RobotStateYearly.get().getLatestFieldRobotPose().getX() > 12.512548;
  }

  /* ---- VELOCITY TABLES ---- */

  /**
   * @return The current velocity table to use for aiming
   */
  private OptimalVelocityTable getCurrentVelocityTable() {

    // Check if there are no robot pose measurements, mostly applicable at start of program runtime
    if (RobotStateYearly.get().getLatestFieldRobotPose() == null) {
      return passingTableSupplier.get();
    }

    if (inShootingZone.getAsBoolean()) {
      return shootingHubTable;
    } else {
      return passingTableSupplier.get();
    }
  }

  /**
   * @return a command that sets the passing position to the outpost
   */
  public Command createSetPassingPoseOutpostCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingOutpostTable;
        });
  }

  /**
   * @return a command that sets the passing position to the left
   */
  public Command createSetPassingPoseLeftCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingLeftTable;
        });
  }

  /**
   * @return a command that sets the passing position to the middle
   */
  public Command createSetPassingPoseMiddleCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingMiddleTable;
        });
  }

  /**
   * @return a command that sets the passing position to the right
   */
  public Command createSetPassingPoseRightCommand() {
    return new InstantCommand(
        () -> {
          passingTableSupplier = () -> passingRightTable;
        });
  }

  /**
   * @return the current optimal yaw to shoot to the goal position
   */
  private Rotation2d getCurrentYaw() {
    return getCurrentVelocityTable()
        .getFuelInitVelocityRotation3d(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds())
        .toRotation2d();
  }

  /**
   * @return the current optimal pitch to shoot to the goal position
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
   * @return the current optimal speed to shoot to the goal position
   */
  private double getCurrentSpeed() {
    return getCurrentVelocityTable()
        .getFuelInitVelocityMagnitude(
            RobotStateYearly.get().getLatestFieldRobotPose(),
            RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
  }

  /* ---- HOOD COMMANDS ---- */

  /**
   * @return a command that sets the hood goal angle to the optimal shooting pitch
   */
  public Command createHoodTowardsGoalCommand() {
    return hood.smartPositionSetpointCommand(() -> getCurrentPitch());
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return true if hood is within 10 degrees of target position, false otherwise
   */
  public boolean isHoodNearGoal() {
    double tolerance = RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig().kAutoAimLeniance;
    return MathUtil.isNear(hood.getCurrentPosition(), getCurrentPitch(), tolerance);
  }

  /* ---- TURRET COMMANDS ---- */

  /**
   * @return Robot-Relative angle of the otimal yaw
   */
  private double getTurretTowardsGoalFromRobotPose() {
    double targetRotation =
        getCurrentYaw()
            .minus(RobotStateYearly.get().getLatestFieldRobotPose().getRotation())
            .getDegrees();

    // Add 180 to turret rotation because 'forward' according to the turret is actually 180 degrees
    // offset from what forward is
    //targetRotation += 180;

    // Because of the way the the auto aim tables are set up, need to rotate turret 180 when on red
    // alliance
    if (RobotRuntimeConstants.isRedAlliance()) {
      targetRotation += 180;
    }

    return MathUtil.inputModulus(targetRotation, 0, 360);
  }

  /**
   * @return a command that sets the turret goal angle to the optimal shooting yaw
   */
  public Command createTurretTowardsGoalCommand() {
    return turret.smartPositionSetpointCommand(() -> getTurretTowardsGoalFromRobotPose());
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return True if turret is within 10 degrees of goal position, false otherwise
   */
  public boolean isTurretNearGoal() {
    double tolerance = RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig().kAutoAimLeniance;
    return MathUtil.isNear(
        turret.getCurrentPosition(), getTurretTowardsGoalFromRobotPose(), tolerance);
  }

  /* ---- FLYWHEEL COMMANDS ---- */

  /**
   * @return a command that sets the flywheel goal velocity to the optimal shooting velocity
   */
  public Command createFlywheelGoalSpeedCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> getCurrentSpeed());
  }

  /**
   * CURRENTLY EMPTY
   *
   * @return a command that sets the flywheel to the idle speed
   */
  public Command createFlywheelIdleSpeedCommand() {
    return new Command() {};
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return True if the flywheel is within 2 meters per second of goal velocity, false otherwise
   */
  public boolean isFlywheelNearGoal() {
    double tolerance =
        RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration().kAutoAimLeniance;
    return MathUtil.isNear(flywheel.getCurrentVelocity(), getCurrentSpeed(), tolerance);
  }

  /* ---- FUEL SHOOTING FUNCTIONS ---- */

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return true if shooter is near goal position, see subsystem specific commands for actual
   *     values
   */
  public boolean isShooterNearGoal() {
    return isFlywheelNearGoal() && isHoodNearGoal() && isTurretNearGoal();
  }

  /**
   * @return A command that begins the fuel shooting process
   */
  public Command createShootFuelCommand() {

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return Commands.none();
      case REPLAY:

      case REAL:

      default:
        return Commands.none(); // Indexer handles fuel supplying
    }
  }
}
